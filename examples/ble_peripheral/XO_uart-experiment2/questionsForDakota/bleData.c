void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */



void update_pwm(int16_t dutyR, int16_t dutyG, int16_t dutyB)
{
    
    seq_values.channel_0 = m_top-dutyR;
    seq_values.channel_1 = m_top-dutyG;
    seq_values.channel_2 = m_top-dutyB;
    

    //seq_values = duty_cycle;

    //nrf_drv_pwm_simple_playback(&m_pwm0, &m_seq, 1, 0);
    //nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1,  NRF_DRV_PWM_FLAG_LOOP);
}
/**@snippet [Controlling the LEDs */
