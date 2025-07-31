/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */



/****************************************************************************
*
* This file is for Classic Bluetooth device and service discovery Demo.
*
****************************************************************************/

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "lcd.h"
#include "driver/i2c.h"
#include "blutooth_scan_header.h"
EventGroupHandle_t Accept_for_http_tast_event_group;
static EventGroupHandle_t s_wifi_event_group;
//EventGroupHandle_t Accept_for_gap_discovery_tast_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define CHECK_WIFI_BIT     1 << 6
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
//#define CHECK_BLUE_GAP_STOP_BIT     1 << 7

void Check_Wifi(void);

static const char *TAGG = "wifi station";

static int s_retry_num = 0;

#define BUTTON_PIN GPIO_NUM_0 
#define MAX_NAMES 4
#define MAX_NAME_LENGTH 100

#define WEB_SERVER "192.168.1.71"
#define WEB_PORT "1602"
#define WEB_PATH "/fromesp"
#define ACCEPT_HTTP_TASK 1 << 5

char requestWifi[300];
char requsetNotWifi[1000];
char buffMacNotWifi[30][18];
char buffTime[30][18];
char time_str[18];
char dungdung[18];
int k =0;
//int wifi_disconnect = 0;

char names[MAX_NAMES][MAX_NAME_LENGTH];
uint8_t Flag = 0;
static const char *TAG = "example";

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM_0;

i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void process_names(char *recv_buf) {
    char part1[100];
    char part2[50];

    split_text(recv_buf, part1, part2);
    analyze_string(part2, names);
}

 static void *createReq(char (*buffMac)[100], char *request) {
    // Create JSON string
    char Buff[256];
    memset(Buff, 0, sizeof(Buff));
    int Len = snprintf(Buff, sizeof(Buff), "{\"mac1\":\"%s\",\"mac2\":\"%s\",\"mac3\":\"%s\",\"mac4\":\"%s\"}",
                       buffMac[0], buffMac[1], buffMac[2], buffMac[3]);

    // HTTP request format with JSON body
    const char *REQUEST_FORMAT = "POST %s HTTP/1.1\r\n"
                                 "Host: %s:%s\r\n"
                                 "Content-Type: application/json\r\n"
                                 "Content-Length: %d\r\n"
                                 "\r\n"
                                 "%s";

    // Create the full HTTP request
    snprintf(request, 512, REQUEST_FORMAT, WEB_PATH, WEB_SERVER, WEB_PORT, Len, Buff);
    return request;
}

static char *createReqNotWifi(char (*Mac)[30], char (*Time)[30],char *request, int k) {
    // Tạo chuỗi JSON
    char Buffer[512]; // Tăng kích thước Buffer để đảm bảo đủ cho chuỗi JSON và yêu cầu HTTP
    memset(Buffer, 0, sizeof(Buffer));

    strcat(Buffer, "{");
    for (int i = 0; i <= k; i++) {
        char temp[50];
        snprintf(temp, sizeof(temp), "\"%s\":\"%s\"", Mac[i], Time[i]);
        strcat(Buffer, temp);
        if (i < k) {
            strcat(Buffer, ",");
        }
    }
    strcat(Buffer, "}");

    int Len = strlen(Buffer);

    // Định dạng yêu cầu HTTP với nội dung JSON
    const char *REQUEST_FORMAT = "POST /promespNotwifi HTTP/1.1\r\n"
                                 "Host: %s:%s\r\n"
                                 "Content-Type: application/json\r\n"
                                 "Content-Length: %d\r\n"
                                 "\r\n"
                                 "%s";
    snprintf(request, 200 + Len, REQUEST_FORMAT, WEB_SERVER, WEB_PORT, Len, Buffer);
    return request;
}


    
static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[400];

    while(1) {
         ESP_LOGI("HTTP Task", "test");
        vTaskDelay(1000/portTICK_PERIOD_MS);
        EventBits_t Accept_for_http_task_event_bit = xEventGroupWaitBits(Accept_for_http_tast_event_group,ACCEPT_HTTP_TASK,pdFALSE,pdFALSE, portMAX_DELAY);
        
        if(Accept_for_http_task_event_bit & ACCEPT_HTTP_TASK){
            esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 0x02, 0);
            ESP_LOGI("HTTP ", "Running into HTTP TASK");
            vTaskDelay(1000/portTICK_PERIOD_MS);
        // vTaskDelay(2000/portTICK_PERIOD_MS);

        if (n == 0) {
        vTaskDelay(2000/portTICK_PERIOD_MS);
        continue;
        }

        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);
    
    
        createReq(buffMac,requestWifi);
    

        if (write(s, requestWifi, strlen(requestWifi)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        memset(buffMac,0,sizeof(buffMac));
        memset(requestWifi,0,sizeof(requestWifi));
        n = 0;
      
        
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 1;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");


        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            ESP_LOGI(TAG,"%d",r);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }

            process_names(recv_buf);
            ESP_LOGI(TAG,"------------%s\n",recv_buf);
            int k =0 ;
            for (int i = 0; i < MAX_NAMES; i++) {

                if (names[i][0] != '\0') {
                 k++;
                    
                   
                }
            } 

            switch (k)
            {
            case 0:
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                break;
            case 1:
                lcd_put_cur(0,4);
                lcd_send_string(names[0]);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                break;
            case 2:
                lcd_put_cur(0,4);
                lcd_send_string(names[0]);
                lcd_put_cur(1,4);
                 lcd_send_string(names[1]);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                break;
            case 3:
                lcd_put_cur(0,4);
                lcd_send_string(names[0]);
                lcd_put_cur(1,4);
                lcd_send_string(names[1]);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                lcd_clear();
                lcd_put_cur(0,4);
                lcd_send_string(names[2]);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
            case 4:
                lcd_put_cur(0,4);
                lcd_send_string(names[0]);
                lcd_put_cur(1,4);
                lcd_send_string(names[1]);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                lcd_clear();
                lcd_put_cur(0,4);
                lcd_send_string(names[2]);
                lcd_put_cur(1,4);
                lcd_send_string(names[3]);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
            
            default:
                break;
            }
            lcd_clear();
             


        } while(r > 0);
        

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        
        }

    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAGG, "Khoi tao thanh cong staion");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
       if (s_retry_num < 3) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
           xEventGroupClearBits(Accept_for_http_tast_event_group, ACCEPT_HTTP_TASK);
           xEventGroupSetBits(Accept_for_http_tast_event_group, CHECK_WIFI_BIT);
        } else {
           xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
        // cho bien int test lên 1, viet ơ day
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAGG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAGG, "Tu" );
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(Accept_for_http_tast_event_group, ACCEPT_HTTP_TASK);
        xEventGroupClearBits(Accept_for_http_tast_event_group, CHECK_WIFI_BIT);
        // b1: kiem tra xem bien test bang 1 thi gui
        // b2 : gui xong clear bien test ve 0 luon (viet trong ham gui)
        // phan gui nay o viet o ngoai nhe
        //-> test chỉ len 1 khi mat mang. có mạng gửi xong nó về 0 luôn, nên sẽ chỉ gửi 1 lần. 
        //Ô có thể đặt trong main hoặc tạo task tuỳ nhes
        const struct addrinfo hints = {
                .ai_family = AF_INET,
                .ai_socktype = SOCK_STREAM,
            };
            struct addrinfo *res;
            struct in_addr *addr;
            int s, r;
            char recv_buf[400];

             ESP_LOGI("sendData Not wifi to server************************************", "test");
            
             int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

                if(err != 0 || res == NULL) {
                    ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
          
                }

                addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
                ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

                s = socket(res->ai_family, res->ai_socktype, 0);
                if(s < 0) {
                    ESP_LOGE(TAG, "... Failed to allocate socket.");
                    freeaddrinfo(res);
            
                }
                ESP_LOGI(TAG, "... allocated socket");

                if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
                    ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
                    close(s);
                    freeaddrinfo(res);
                    vTaskDelay(4000 / portTICK_PERIOD_MS);
                }

                ESP_LOGI(TAG, "... connected");
                freeaddrinfo(res);
                ESP_LOGI(TAG,"---------------------------------------------------%d",k);
                
                createReqNotWifi(buffMacNotWifi,buffTime,requsetNotWifi,k-1);
                
                if (write(s,requsetNotWifi, strlen(requsetNotWifi)) < 0) {
                    ESP_LOGE(TAG, "... socket send failed");
                    close(s);
                    vTaskDelay(4000 / portTICK_PERIOD_MS);
                  }
                memset(buffMacNotWifi,0,sizeof(buffMacNotWifi));
                memset(buffTime,0,sizeof(buffTime));
                memset(requsetNotWifi,0,sizeof(requsetNotWifi));
                k=0;
            
            
                
                ESP_LOGI(TAG, "... socket send success");

                struct timeval receiving_timeout;
                receiving_timeout.tv_sec = 1;
                receiving_timeout.tv_usec = 0;
                if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                        sizeof(receiving_timeout)) < 0) {
                    ESP_LOGE(TAG, "... failed to set socket receiving timeout");
                    close(s);
                    vTaskDelay(4000 / portTICK_PERIOD_MS);
                }
                ESP_LOGI(TAG, "... set socket receiving timeout success");

                do {
                    bzero(recv_buf, sizeof(recv_buf));
                    r = read(s, recv_buf, sizeof(recv_buf)-1);
                    ESP_LOGI(TAG,"%d",r);
                    for(int i = 0; i < r; i++) {
                        putchar(recv_buf[i]);
                    }
                } while(r > 0);
                

                ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
                close(s);
             
    }
}
static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    //ESP_ERROR_CHECK(esp_netif_init());

    //ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
     wifi_config_t wifi_config = {
        .sta = {
            .ssid = "ESP",
            .password = "12345678",
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            // .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            // .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            // .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    
    // ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    // ESP_ERROR_CHECK(esp_wifi_start() );

    // ESP_LOGI(TAGG, "wifi_init_sta finished.");

}


void Check_Wifi(void){
    while(1){  
        ESP_LOGI("Check wifi task", "test");
        vTaskDelay(1000/portTICK_PERIOD_MS);
        EventBits_t event_bit = xEventGroupWaitBits(Accept_for_http_tast_event_group,CHECK_WIFI_BIT,pdFALSE,pdFALSE,portMAX_DELAY);
        if(event_bit & CHECK_WIFI_BIT){
            esp_wifi_connect();
            vTaskDelay(1000/portTICK_PERIOD_MS);
            ESP_LOGI(TAGG, "Tu dang khoi dong lai");
            esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 0x02, 0);
                if (strcmp(Mac_Lost_Wifi, "\0") != 0) {
                     strcpy(dungdung,Mac_Lost_Wifi);
                printf("****************************************************************************************************************");
                    // Kiểm tra xem chuỗi đã tồn tại trong mảng buffMacNotWifi chưa
                    bool exists = false;
                    for (int i = 0; i <= k; i++) {
                        printf("\n\n\n\n%s\n\n\n\n",buffMacNotWifi[i]);

                      //  tim trong buff xem co chua
                        if (strcmp(buffMacNotWifi[i], dungdung) == 0) {


                            memset(dungdung,0,sizeof(dungdung));//

                              ESP_LOGI("ddddddddddddddddddddddddddddddddddddddddddddd", "test");

                            // Chuỗi đã tồn tại, không cần thêm vào mảng, đánh dấu và kết thúc hàm
                            exists = true;
                            break;
                        }
                    }

                    if (!exists && k < 30) {
                        printf("_-----------------------------%d",k);
                        // Thêm Mac_Lost_Wifi và thời gian vào mảng
                        strcpy(buffMacNotWifi[k], dungdung);
                        memset(dungdung,0,sizeof(dungdung));
                        strcpy(buffTime[k], "aaaaaaaaaa");
                        // Tăng chỉ số k lên 1
                        k++;
                    } 
        }
    }
        // Reset watchdog nhiệm vụ
        // viet ơ đây
    }
}
void Check_Blue(void){
    while(1){  
        ESP_LOGI("Check Blue task", "test");
        vTaskDelay(1000/portTICK_PERIOD_MS);
        //EventBits_t event_bit2 = xEventGroupWaitBits(Accept_for_blue_discovery_event_group,CHECK_BLU_BIT,pdFALSE,pdFALSE,portMAX_DELAY);
        //if(event_bit2 & CHECK_BLU_BIT){
            esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 0x04, 0);
            vTaskDelay(6000/portTICK_PERIOD_MS);
             ESP_LOGI(TAGG, "Tu dang quet lai");      
           // vTaskSuspend(xTaskToSuspend);
       // }
        //vTaskDelay(4000/portTICK_PERIOD_MS);
    }
        // Reset watchdog nhiệm vụ
        // viet ơ đây
    }
void app_main(void){
    // char buffMac[5][18];

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    
    Accept_for_http_tast_event_group = xEventGroupCreate();  
    Accept_for_blue_discovery_event_group = xEventGroupCreate(); 



     ESP_ERROR_CHECK(esp_netif_init());
     ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_sta();
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    lcd_init();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(GAP_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(GAP_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

   // esp_bluedroid_status_t bluedroid_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(GAP_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(GAP_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    bt_app_gap_start_up();

   // Accept_for_gap_discovery_tast_event_group = xEventGroupCreate();

    ESP_LOGI("Check", "1");
    xTaskCreate((TaskFunction_t)Check_Wifi, "Check_Wifi", 4096, NULL, 2,NULL);

    ESP_LOGI("Check", "2");
    xTaskCreate((TaskFunction_t)http_get_task, "http_get_task", 4096, NULL, 2, NULL);
    ESP_LOGI("Check", "3");
   xTaskCreate((TaskFunction_t)Check_Blue, "Check_Blue", 4096, NULL, 2, xTaskToSuspend);
  // xTaskCreate((TaskFunction_t)Check_Blue, "Check_Blue", 4096, NULL, 2, NULL);
   ESP_LOGI("Check", "4");
}
