
#include "blutooth_scan_header.h"

 app_gap_cb_t m_dev_info;
 app_gap_cb_t *p_dev;
char buffMac[4][18];
char Mac_Lost_Wifi[18] = {0};
int n = 0;

EventGroupHandle_t Accept_for_blue_discovery_event_group;
TaskHandle_t xTaskToSuspend;
 char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    // request = createReq(str);
    
   return str;
}

 char *uuid2str(esp_bt_uuid_t *uuid, char *str, size_t size)
{
    if (uuid == NULL || str == NULL) {
        return NULL;
    }

    if (uuid->len == 2 && size >= 5) {
        sprintf(str, "%04x", uuid->uuid.uuid16);
    } else if (uuid->len == 4 && size >= 9) {
        sprintf(str, "%08"PRIx32, uuid->uuid.uuid32);
    } else if (uuid->len == 16 && size >= 37) {
        uint8_t *p = uuid->uuid.uuid128;
        sprintf(str, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                p[15], p[14], p[13], p[12], p[11], p[10], p[9], p[8],
                p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]);
    } else {
        return NULL;
    }

    return str;
}

 bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

 void update_device_info(esp_bt_gap_cb_param_t *param)
{
    printf(">>>  ugsk2,3txi3tx3xc\n");
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    uint8_t *bdname = NULL;
    uint8_t bdname_len = 0;
    uint8_t *eir = NULL;
    uint8_t eir_len = 0;

    esp_bt_gap_dev_prop_t *p;
    
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            ESP_LOGI(GAP_TAG, "--Class of Device: 0x%"PRIx32, cod);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            ESP_LOGI(GAP_TAG, "--RSSI: %"PRId32, rssi);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
            bdname_len = (p->len > ESP_BT_GAP_MAX_BDNAME_LEN) ? ESP_BT_GAP_MAX_BDNAME_LEN :
                          (uint8_t)p->len;
            bdname = (uint8_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR: {
            eir_len = p->len;
            eir = (uint8_t *)(p->val);
            break;
        }
        default:
            break;
        }
    }

    /* search for device with Major device type "PHONE" or "Audio/Video" in COD */
    p_dev = &m_dev_info;
    // if (p_dev->dev_found) {
    //     ESP_LOGI(GAP_TAG, "Device found: ???");
    //     return;
    // }

    if (!esp_bt_gap_is_valid_cod(cod) ||
        (!(esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_PHONE) &&
         !(esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_AV))) {
        return;
    }

    /* Chỉ xử lý các thiết bị có giá trị RSSI trong khoảng -50 đến -20 */
    if (rssi < -50 || rssi > -10) {
    
       m_dev_info.dev_found = false ;
        return;
    }

    memcpy(p_dev->bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
    p_dev->dev_found = true;

    bda2str(param->disc_res.bda, bda_str, 18);
   strcpy(Mac_Lost_Wifi, bda_str);
    
   for ( int i = 0; i < 4; i++) {
    printf("%s\n", buffMac[i]);
   }
   printf("%d",sizeof(buffMac));


for ( int i = 0; i < 4; i++) {
    // Kiểm tra xem chuỗi đã tồn tại trong mảng buffMac chưa
    if (strcmp(buffMac[i], bda_str) == 0) {
        // Chuỗi đã tồn tại, không cần thêm vào mảng, kết thúc hàm
        return;
    }
}

// Nếu không tìm thấy chuỗi trong mảng buffMac, thêm chuỗi vào mảng
if ( n < 4) {
    strcpy(buffMac[n], bda_str);
    // Tăng chỉ số n lên 1
    n++;
}



    p_dev->cod = cod;
    p_dev->rssi = rssi;
    if (bdname_len > 0) {
        memcpy(p_dev->bdname, bdname, bdname_len);
        p_dev->bdname[bdname_len] = '\0';
        p_dev->bdname_len = bdname_len;
    }
    if (eir_len > 0) {
        memcpy(p_dev->eir, eir, eir_len);
        p_dev->eir_len = eir_len;
    }

    if (p_dev->bdname_len == 0) {
        get_name_from_eir(p_dev->eir, p_dev->bdname, &p_dev->bdname_len);
    }

    p_dev->state = APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE;
    ESP_LOGI(GAP_TAG, "Cancel device discovery ...");
}

 void bt_app_gap_init(void)
{
    app_gap_cb_t *p_dev = &m_dev_info;
    memset(p_dev, 0, sizeof(app_gap_cb_t));

    p_dev->state = APP_GAP_STATE_IDLE;
}

 void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    app_gap_cb_t *p_dev = &m_dev_info;
    char bda_str[18];
    char uuid_str[37];

    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        update_device_info(param);
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(GAP_TAG, "Device discovery stopped.");
           // xEventGroupSetBits(Accept_for_blue_discovery_event_group, CHECK_BLU_BIT);
            ///vTaskResume(xTaskToSuspend);
            //esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 0x03, 0);
            xEventGroupSetBits(Accept_for_blue_discovery_event_group, CHECK_BLU_BIT);
          // xEventGroupSetBits(Accept_for_blue_discovery_event_group, CHECK_BLU_BIT);
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(GAP_TAG, "Discovery started.");
            //xEventGroupSetBits(Accept_for_blue_discovery_event_group, CHECK_BLU_BIT);
            //esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 0x03, 0);
        }
        break;
    }
    case ESP_BT_GAP_RMT_SRVCS_EVT: {
        if (memcmp(param->rmt_srvcs.bda, p_dev->bda, ESP_BD_ADDR_LEN) == 0 &&
                p_dev->state == APP_GAP_STATE_SERVICE_DISCOVERING) {
            p_dev->state = APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE;
            if (param->rmt_srvcs.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(GAP_TAG, "Services for device %s found",  bda2str(p_dev->bda, bda_str, 18));
                for (int i = 0; i < param->rmt_srvcs.num_uuids; i++) {
                    esp_bt_uuid_t *u = param->rmt_srvcs.uuid_list + i;
                    ESP_LOGI(GAP_TAG, "--%s", uuid2str(u, uuid_str, 37));
                }
            } else {
                ESP_LOGI(GAP_TAG, "Services for device %s not found",  bda2str(p_dev->bda, bda_str, 18));
            }
        }
        //xEventGroupSetBits(Accept_for_blue_discovery_event_group, CHECK_BLU_BIT);
        break;

    }
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
    default: {
        ESP_LOGI(GAP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

 void bt_app_gap_start_up(void)
{
    /* register GAP callback function */
    esp_bt_gap_register_callback(bt_app_gap_cb);

    char *dev_name = "ESP_GAP_INQRUIY";
    esp_bt_dev_set_device_name(dev_name);

    /* set discoverable and connectable mode, wait to be connected */
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    /* inititialize device information and status */
    // bt_app_gap_init();

    /* start to discover nearby Bluetooth devices */
    app_gap_cb_t *p_dev = &m_dev_info;
    p_dev->state = APP_GAP_STATE_DEVICE_DISCOVERING;
    //esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 0x01, 2);
}

