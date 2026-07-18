local pwm_last = 0

function update()
    local pwm_new = SRV_Channels:get_output_pwm_chan(15)

    if pwm_new ~= pwm_last then
        pwm_last = pwm_new 

        if pwm_new == 1100 then
            camera:change_setting(0, 0, 0)
            gcs:send_text(6, "Thermal Palette = WhiteHot")

        elseif pwm_new == 1200 then
            camera:change_setting(0, 0, 10)
            gcs:send_text(6, "Thermal Palette = BlackHot")
		
		elseif pwm_new == 1300 then
            camera:change_setting(0, 0, 3)
            gcs:send_text(6, "Thermal Palette = Ironbow")
		
		elseif pwm_new == 1400 then
            camera:change_setting(0, 0, 4)
            gcs:send_text(6, "Thermal Palette = Rainbow")
			
		elseif pwm_new == 1500 then
            camera:change_setting(0, 0, 7)
            gcs:send_text(6, "Thermal Palette = RedHot")
			
        end

    end
    
    return update, 100
end

gcs:send_text(6, "started scripting")
return update, 3000