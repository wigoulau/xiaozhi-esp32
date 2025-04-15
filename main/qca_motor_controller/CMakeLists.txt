idf_component_register(SRCS "qca_preset_action.cc" "qca_pwm_helper.cc" "qca_motor_controller.cc" "qca_preset_action.c" "qca_pwm_helper.c" "qca_motor_controller.c"
                    REQUIRES driver esp_timer
                    INCLUDE_DIRS "include")
)
