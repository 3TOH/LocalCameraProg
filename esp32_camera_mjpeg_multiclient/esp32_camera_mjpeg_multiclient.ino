/*

  This is a simple MJPEG streaming webserver implemented for AI-Thinker ESP32-CAM
  and ESP-EYE modules.
  This is tested to work with VLC and Blynk video widget and can support up to 10
  simultaneously connected streaming clients.
  Simultaneous streaming is implemented with FreeRTOS tasks.

  Inspired by and based on this Instructable: $9 RTSP Video Streamer Using the ESP32-CAM Board
  (https://www.instructables.com/id/9-RTSP-Video-Streamer-Using-the-ESP32-CAM-Board/)

  Board: AI-Thinker ESP32-CAM or ESP-EYE
  Compile as:
   ESP32 Dev Module
   CPU Freq: 240
   Flash Freq: 80
   Flash mode: QIO
   Flash Size: 4Mb
   Patrition: Minimal SPIFFS
   PSRAM: Enabled
*/

// ESP32 has two cores: APPlication core and PROcess core (the one that runs ESP32 SDK stack)
#define APP_CPU 1
#define PRO_CPU 0

#include "OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

#include <esp_task_wdt.h>
#include <EEPROM.h>

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#define EEPROM_SIZE 1

#include "camera_pins.h"

/*
  Next one is an include with wifi credentials.
  This is what you need to do:

  1. Create a file called "home_wifi_multi.h" in the same folder   OR   under a separate subfolder of the "libraries" folder of Arduino IDE. (You are creating a "fake" library really - I called it "MySettings").
  2. Place the following text in the file:
  #define SSID1 "replace with your wifi ssid"
  #define PWD1 "replace your wifi password"
  3. Save.

  Should work then
*/
//#include "home_wifi_multi.h"
#define SSID1 "PS1"
#define PWD1 "98909890Aa"


OV2640 cam;

WebServer server(80);

// ===== rtos task handles =========================
// Streaming is implemented with 3 tasks:
TaskHandle_t tMjpeg;   // handles client connections to the webserver
TaskHandle_t tCam;     // handles getting picture frames from the camera and storing them locally
TaskHandle_t tStream;  // actually streaming frames to all connected clients

// frameSync semaphore is used to prevent streaming buffer as it is replaced with the next frame
SemaphoreHandle_t frameSync = NULL;

// Queue stores currently connected clients to whom we are streaming
QueueHandle_t streamingClients;

// We will try to achieve 25 FPS frame rate
const int FPS = 14;

// We will handle web client requests every 50 ms (20 Hz)
const int WSINTERVAL = 100;

// ======== Server Connection Handler Task ==========================
void mjpegCB(void* pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(WSINTERVAL);

  // Creating frame synchronization semaphore and initializing it
  frameSync = xSemaphoreCreateBinary();
  xSemaphoreGive( frameSync );

  // Creating a queue to track all connected clients
  streamingClients = xQueueCreate( 10, sizeof(WiFiClient*) );

  //=== setup section  ==================

  //  Creating RTOS task for grabbing frames from the camera
  xTaskCreatePinnedToCore(
    camCB,        // callback
    "cam",        // name
    4096,         // stacj size
    NULL,         // parameters
    2,            // priority
    &tCam,        // RTOS task handle
    APP_CPU);     // core

  //  Creating task to push the stream to all connected clients
  xTaskCreatePinnedToCore(
    streamCB,
    "strmCB",
    4 * 1024,
    NULL, //(void*) handler,
    2,
    &tStream,
    APP_CPU);

  //  Registering webserver handling routines
  server.on("/mjpeg", HTTP_GET, handleJPGSstream);
  server.on("/jpg", HTTP_GET, handleJPG);
  server.on("/config", HTTP_GET, handleConfig);
  server.onNotFound(handleNotFound);

  //  Starting webserver
  server.begin();

  //=== loop() section  ===================
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    server.handleClient();

    //  After every server client handling request, we let other tasks run and then pause
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// Commonly used variables:
volatile size_t camSize;    // size of the current frame, byte
volatile char* camBuf;      // pointer to the current frame


// ==== RTOS task to grab frames from the camera =========================
void camCB(void* pvParameters) {

  TickType_t xLastWakeTime;

  //  A running interval associated with currently desired frame rate
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / FPS);

  // Mutex for the critical section of swithing the active frames around
  portMUX_TYPE xSemaphore = portMUX_INITIALIZER_UNLOCKED;

  //  Pointers to the 2 frames, their respective sizes and index of the current frame
  char* fbs[2] = { NULL, NULL };
  size_t fSize[2] = { 0, 0 };
  int ifb = 0;

  //=== loop() section  ===================
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {

    //  Grab a frame from the camera and query its size
    cam.run();
    size_t s = cam.getSize();

    //  If frame size is more that we have previously allocated - request  125% of the current frame space
    if (s > fSize[ifb]) {
      fSize[ifb] = s * 4 / 3;
      fbs[ifb] = allocateMemory(fbs[ifb], fSize[ifb]);
    }

    //  Copy current frame into local buffer
    char* b = (char*) cam.getfb();
    memcpy(fbs[ifb], b, s);

    //  Let other tasks run and wait until the end of the current frame rate interval (if any time left)
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    //  Only switch frames around if no frame is currently being streamed to a client
    //  Wait on a semaphore until client operation completes
    xSemaphoreTake( frameSync, portMAX_DELAY );

    //  Do not allow interrupts while switching the current frame
    portENTER_CRITICAL(&xSemaphore);
    camBuf = fbs[ifb];
    camSize = s;
    ifb++;
    ifb &= 1;  // this should produce 1, 0, 1, 0, 1 ... sequence
    portEXIT_CRITICAL(&xSemaphore);

    //  Let anyone waiting for a frame know that the frame is ready
    xSemaphoreGive( frameSync );

    //  Technically only needed once: let the streaming task know that we have at least one frame
    //  and it could start sending frames to the clients, if any
    xTaskNotifyGive( tStream );

    //  Immediately let other (streaming) tasks run
    taskYIELD();

    //  If streaming task has suspended itself (no active clients to stream to)
    //  there is no need to grab frames from the camera. We can save some juice
    //  by suspedning the tasks
    if ( eTaskGetState( tStream ) == eSuspended ) {
      vTaskSuspend(NULL);  // passing NULL means "suspend yourself"
    }
  }
}


// ==== Memory allocator that takes advantage of PSRAM if present =======================
char* allocateMemory(char* aPtr, size_t aSize) {

  //  Since current buffer is too smal, free it
  if (aPtr != NULL) free(aPtr);


  size_t freeHeap = ESP.getFreeHeap();
  char* ptr = NULL;

  // If memory requested is more than 2/3 of the currently free heap, try PSRAM immediately
  if ( aSize > freeHeap * 2 / 3 ) {
    if ( psramFound() && ESP.getFreePsram() > aSize ) {
      ptr = (char*) ps_malloc(aSize);
    }
  }
  else {
    //  Enough free heap - let's try allocating fast RAM as a buffer
    ptr = (char*) malloc(aSize);

    //  If allocation on the heap failed, let's give PSRAM one more chance:
    if ( ptr == NULL && psramFound() && ESP.getFreePsram() > aSize) {
      ptr = (char*) ps_malloc(aSize);
    }
  }

  // Finally, if the memory pointer is NULL, we were not able to allocate any memory, and that is a terminal condition.
  if (ptr == NULL) {
    ESP.restart();
  }
  return ptr;
}


// ==== STREAMING ======================================================
const char HEADER[] = "HTTP/1.1 200 OK\r\n" \
                      "Access-Control-Allow-Origin: *\r\n" \
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);


// ==== Handle connection request from clients ===============================
void handleJPGSstream(void)
{
  //  Can only acommodate 10 clients. The limit is a default for WiFi connections
  if ( !uxQueueSpacesAvailable(streamingClients) ) return;


  //  Create a new WiFi Client object to keep track of this one
  WiFiClient* client = new WiFiClient();
  *client = server.client();

  //  Immediately send this client a header
  client->write(HEADER, hdrLen);
  client->write(BOUNDARY, bdrLen);

  // Push the client to the streaming queue
  xQueueSend(streamingClients, (void *) &client, 0);

  // Wake up streaming tasks, if they were previously suspended:
  if ( eTaskGetState( tCam ) == eSuspended ) vTaskResume( tCam );
  if ( eTaskGetState( tStream ) == eSuspended ) vTaskResume( tStream );
}


// ==== Actually stream content to all connected clients ========================
void streamCB(void * pvParameters) {
  char buf[16];
  TickType_t xLastWakeTime;
  TickType_t xFrequency;

  //  Wait until the first frame is captured and there is something to send
  //  to clients
  ulTaskNotifyTake( pdTRUE,          /* Clear the notification value before exiting. */
                    portMAX_DELAY ); /* Block indefinitely. */

  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    // Default assumption we are running according to the FPS
    xFrequency = pdMS_TO_TICKS(1000 / FPS);

    //  Only bother to send anything if there is someone watching
    UBaseType_t activeClients = uxQueueMessagesWaiting(streamingClients);
    if ( activeClients ) {
      // Adjust the period to the number of connected clients
      xFrequency /= activeClients;

      //  Since we are sending the same frame to everyone,
      //  pop a client from the the front of the queue
      WiFiClient *client;
      xQueueReceive (streamingClients, (void*) &client, 0);

      //  Check if this client is still connected.

      if (!client->connected()) {
        //  delete this client reference if s/he has disconnected
        //  and don't put it back on the queue anymore. Bye!
        delete client;
      }
      else {

        //  Ok. This is an actively connected client.
        //  Let's grab a semaphore to prevent frame changes while we
        //  are serving this frame
        xSemaphoreTake( frameSync, portMAX_DELAY );

        client->write(CTNTTYPE, cntLen);
        sprintf(buf, "%d\r\n\r\n", camSize);
        client->write(buf, strlen(buf));
        client->write((char*) camBuf, (size_t)camSize);
        client->write(BOUNDARY, bdrLen);

        // Since this client is still connected, push it to the end
        // of the queue for further processing
        xQueueSend(streamingClients, (void *) &client, 0);

        //  The frame has been served. Release the semaphore and let other tasks run.
        //  If there is a frame switch ready, it will happen now in between frames
        xSemaphoreGive( frameSync );
        taskYIELD();
      }
    }
    else {
      //  Since there are no connected clients, there is no reason to waste battery running
      vTaskSuspend(NULL);
    }
    //  Let other tasks run after serving every client
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}



const char JHEADER[] = "HTTP/1.1 200 OK\r\n" \
                       "Content-disposition: inline; filename=capture.jpg\r\n" \
                       "Content-type: image/jpeg\r\n\r\n";
const int jhdLen = strlen(JHEADER);

// ==== Serve up one JPEG frame =============================================
void handleJPG(void)
{
  WiFiClient client = server.client();

  if (!client.connected()) return;
  cam.run();
  client.write(JHEADER, jhdLen);
  client.write((char*)cam.getfb(), cam.getSize());
}

int getParameterFromUrl(String name)
{
  String var;
  var = server.arg(name);

  if (var.length() == 0)
  {
    return -999999;
  }

  return var.toInt();
}

/*void saveEEPROM(void)
{
  sensor_t *s = esp_camera_sensor_get();

  uint8_t resolution = s->status.framesize;
  EEPROM.write(0, resolution);

  EEPROM.commit();
}*/

// ==== Serve up one JPEG frame =============================================
void handleConfig(void)
{
  WiFiClient client = server.client();

  //if (!client.connected()) return;
  
  sensor_t *s = esp_camera_sensor_get();
  
  int res = 0;

  int value = getParameterFromUrl("resolution");
  if (value != -999999) { res = s->set_framesize(s, (framesize_t)value); }

  value = getParameterFromUrl("quality");
  if (value != -999999) { res = s->set_quality(s, value); }

  value = getParameterFromUrl("contrast");
  if (value != -999999) { res = s->set_contrast(s, value); }

  value = getParameterFromUrl("brightness");
  if (value != -999999) { res = s->set_brightness(s, value); }

  value = getParameterFromUrl("saturation");
  if (value != -999999) { res = s->set_saturation(s, value); }

  value = getParameterFromUrl("gainceiling");
  if (value != -999999) { res = s->set_gainceiling(s, (gainceiling_t)value); }

  value = getParameterFromUrl("colorbar");
  if (value != -999999) { res = s->set_colorbar(s, value); }

  value = getParameterFromUrl("awb");
  if (value != -999999) { res = s->set_whitebal(s, value); }

  value = getParameterFromUrl("agc");
  if (value != -999999) { res = s->set_gain_ctrl(s, value); }

  value = getParameterFromUrl("aec");
  if (value != -999999) { res = s->set_exposure_ctrl(s, value); }

  value = getParameterFromUrl("hmirror");
  if (value != -999999) { res = s->set_hmirror(s, value); }

  value = getParameterFromUrl("vflip");
  if (value != -999999) { res = s->set_vflip(s, value); }

  value = getParameterFromUrl("awb_gain");
  if (value != -999999) { res = s->set_awb_gain(s, value); }

  value = getParameterFromUrl("agc_gain");
  if (value != -999999) { res = s->set_agc_gain(s, value); }

  value = getParameterFromUrl("aec_value");
  if (value != -999999) { res = s->set_aec_value(s, value); }

  value = getParameterFromUrl("aec2");
  if (value != -999999) { res = s->set_aec2(s, value); }

  value = getParameterFromUrl("dcw");
  if (value != -999999) { res = s->set_dcw(s, value); }

  value = getParameterFromUrl("bpc");
  if (value != -999999) { res = s->set_bpc(s, value); }

  value = getParameterFromUrl("wpc");
  if (value != -999999) { res = s->set_wpc(s, value); }

  value = getParameterFromUrl("raw_gma");
  if (value != -999999) { res = s->set_raw_gma(s, value); }

  value = getParameterFromUrl("lenc");
  if (value != -999999) { res = s->set_lenc(s, value); }

  value = getParameterFromUrl("special_effect");
  if (value != -999999) { res = s->set_special_effect(s, value); }

  value = getParameterFromUrl("wb_mode");
  if (value != -999999) { res = s->set_wb_mode(s, value); }

  value = getParameterFromUrl("ae_level");
  if (value != -999999) { res = s->set_ae_level(s, value); }

  value = getParameterFromUrl("reset");
  if (value != -999999) { ESP.restart(); }

  //value = getParameterFromUrl("save");
  //if (value != -999999) { saveEEPROM(); }
  
  //client.write(res);
  server.send(200, "text / plain", "OK");
}

const char * htmlMessage = "<html>\n" \
"    <style>\n" \
"        body {\n" \
"            background-color:#1f1f1f;\n" \
"        }\n" \
"        img {\n" \
"            display: block;\n" \
"            margin-left: auto;\n" \
"            margin-right: auto;\n" \
"            max-width: 100%;\n" \
"            max-height: 100%;\n" \
"        }\n" \
"        .inline {\n" \
"            display:inline-block;\n" \
"        }\n" \
"        .centered {\n" \
"            max-width: 100%;\n" \
"            max-height: 100%;\n" \
"        }\n" \
"        .disabled {\n" \
"            opacity: 0.6;\n" \
"            cursor: not-allowed;\n" \
"            visibility: hidden;\n" \
"        }\n" \
"        button, select, input {\n" \
"                background-color: #4eb952;\n" \
"                border: none;\n" \
"                color: white;\n" \
"                padding: 16px 32px;\n" \
"                text-align: center;\n" \
"                text-decoration: none;\n" \
"                display: inline-block;\n" \
"                font-size: 16px;\n" \
"                margin: 4px 2px;\n" \
"                transition-duration: 0.4s;\n" \
"                cursor: pointer;\n" \
"        }\n" \
"        button, select, input {\n" \
"            background-color: #5cd561;\n" \
"            color: black; \n" \
"            border: 2px solid #4CAF50;\n" \
"        }\n" \
"        button:hover, select:hover {\n" \
"            background-color: #4CAF50;\n" \
"            color: white;\n" \
"        }\n" \
"        .slider {\n" \
"            -webkit-appearance: none;\n" \
"            height: 15px;\n" \
"            border-radius: 5px;\n" \
"            background: #d3d3d3;\n" \
"            outline: none;\n" \
"            opacity: 1;\n" \
"        }\n" \
"        .slider::-webkit-slider-thumb {\n" \
"            -webkit-appearance: none;\n" \
"            appearance: none;\n" \
"            width: 25px;\n" \
"            height: 25px;\n" \
"            border-radius: 50%;\n" \
"            background: #04AA6D;\n" \
"            cursor: pointer;\n" \
"        }\n" \
"        .slider::-moz-range-thumb {\n" \
"            width: 25px;\n" \
"            height: 25px;\n" \
"            border-radius: 50%;\n" \
"            background: #04AA6D;\n" \
"            cursor: pointer;\n" \
"        }\n" \
"        p {\n" \
"            color: white;\n" \
"        }\n" \
"    </style>\n" \
"    <body>\n" \
"        <img id='stream_box' src='http://192.168.50.98/mjpeg' />\n" \
"        <button id='unlock_settings'>&#9881;</button>\n" \
"        <button id='change_camera'>&#8634;</button>\n" \
"        <button id='restart_button' class='disabled'>Restart ESP</button>\n" \
"        <select id='resolution_list' class='disabled'>\n" \
"            <option value='0'>96X96 (96x96)</option>\n" \
"            <option value='1'>QQVGA (160x120)</option>\n" \
"            <option value='2'>QCIF (176x144)</option>\n" \
"            <option value='3'>HQVGA (240x176)</option>\n" \
"            <option value='4'>240X240 (240x240)</option>\n" \
"            <option value='5'>QVGA (320x240)</option>\n" \
"            <option value='6'>CIF (400x296)</option>\n" \
"            <option value='7'>HVGA (480x320)</option>\n" \
"            <option value='8'>VGA (640x480)</option>\n" \
"            <option value='9'>SVGA (800x600)</option>\n" \
"            <option value='10'>XGA (1024x768)</option>\n" \
"            <option value='11'>HD (1280x720)</option>\n" \
"            <option value='12'>SXGA (1280x1024)</option>\n" \
"            <option value='13' selected>UXGA (1600x1200)</option>\n" \
"        </select>\n" \
"        <div id='quality_container' class='disabled'>\n" \
"            <input type='range' min='1' max='64' value='30' class='slider' id='quality_slider'>\n" \
"            <p>Quality: <span id='quality_slider_value'></span></p>\n" \
"        </div>\n" \
"    </body>\n" \

"    <script>\n" \
"        const addr = 'http://192.168.50.98/'\n" \
"        const stream_box = document.getElementById('stream_box')\n" \

"        const stream_src1 = 'http://192.168.50.98/mjpeg'\n" \
"        const stream_src2 = 'http://192.168.50.133:4343/mjpeg'\n" \

"        function httpGet(theUrl)\n" \
"        {\n" \
"            var xmlHttp = new XMLHttpRequest();\n" \
"            xmlHttp.onreadystatechange = function() { \n" \
"                if (xmlHttp.readyState == 4 && xmlHttp.status == 200)\n" \
"                    callback(xmlHttp.responseText);\n" \
"            }\n" \
"            xmlHttp.open('GET', theUrl, true);\n" \
"            xmlHttp.send(null);\n" \
"        }\n" \

"        const resetButton = document.getElementById('restart_button')\n" \
"        resetButton.onclick = () => {\n" \
"            httpGet('http://192.168.50.98/config?reset=1')\n" \
"            setTimeout(()=> {\n" \
"                location.reload()\n" \
"            }, 5000)\n" \
"        }\n" \

"        const change_camera = document.getElementById('change_camera')\n" \
"        change_camera.onclick = () => {\n" \
"            if (stream_box.src == stream_src1) {\n" \
"                stream_box.src = stream_src2\n" \
"            } else {\n" \
"                stream_box.src = stream_src1\n" \
"            }\n" \
"        }\n" \

"        counter = 0\n" \
"        const unlock_settings = document.getElementById('unlock_settings')\n" \
"        unlock_settings.onclick = () => {\n" \
"            counter++\n" \
"            if (counter > 4) {\n" \
"                document.getElementById('restart_button').classList.remove('disabled')\n" \
"                document.getElementById('resolution_list').classList.remove('disabled')\n" \
"                document.getElementById('quality_container').classList.remove('disabled')\n" \
"                unlock_settings.classList.add('disabled')\n" \
"            }\n" \
"        }\n" \

"        const resolution_list = document.getElementById('resolution_list')\n" \
"        resolution_list.onchange = () => {\n" \
"            concatStr = addr + 'config?resolution=' + resolution_list.value\n" \
"            httpGet(concatStr)\n" \
"        }\n" \

"        const slider = document.getElementById('quality_slider');\n" \
"        const output = document.getElementById('quality_slider_value');\n" \
"        output.innerHTML = document.getElementById('quality_slider').value;\n" \

"        slider.oninput = () => {\n" \
"            output.innerHTML = slider.value;\n" \
"            httpGet(addr + 'config?quality=' + slider.value)\n" \
"        }\n" \
"    </script>\n" \
"</html>";

// ==== Handle invalid URL requests ============================================
void handleNotFound()
{
  String message = "Server is running!\n\n";
  message = htmlMessage;
  server.send(200, "text / plain", message);
}

#define WDT_TIMEOUT 3

// ==== SETUP method ==================================================================
void setup()
{

  // Setup Serial connection:
  Serial.begin(115200);
  delay(1000); // wait for a second to let Serial connect


  // Configure the camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame parameters: pick one
  //  config.frame_size = FRAMESIZE_UXGA;
  //  config.frame_size = FRAMESIZE_SVGA;
  //  config.frame_size = FRAMESIZE_QVGA;

  //int fr_loaded = EEPROM.read(0);e:\DISKCOPY\Desktop\esp32-cam-mjpeg-multiclient-master\esp32_camera_mjpeg_multiclient\OV2640.h
  //if (fr_loaded < FRAMESIZE_QVGA && fr_loaded > FRAMESIZE_UXGA) fr_loaded = FRAMESIZE_UXGA;
  //Serial.println(fr_loaded);
  config.frame_size = FRAMESIZE_UXGA;
  
  config.jpeg_quality = 20;
  config.fb_count = 2;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  if (cam.init(config) != ESP_OK) {
    Serial.println("Error initializing the camera");
    delay(10000);
    ESP.restart();
  }


  //  Configure and connect to WiFi
  IPAddress ip;

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID1, PWD1);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println("");
  Serial.print("Stream Link: http://");
  Serial.print(ip);
  Serial.println("/mjpeg/1");


  // Start mainstreaming RTOS task
  xTaskCreatePinnedToCore(
    mjpegCB,
    "mjpeg",
    4 * 1024,
    NULL,
    2,
    &tMjpeg,
    APP_CPU);

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
}


void loop() {
  vTaskDelay(1000);

  if (WiFi.status() == WL_CONNECTED)
  {
    esp_task_wdt_reset();
  }
}
