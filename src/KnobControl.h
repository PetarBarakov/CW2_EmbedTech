#include <Arduino.h>

class Knob{

public:

    Knob(uint8_t l_limit, uint8_t h_limit)
    {
        knob_control_data_high_limit = h_limit;
        knob_control_data_low_limit = l_limit;
        knob_control_data_default = 0.5 * (h_limit - l_limit);
        

        knob_control_data = knob_control_data_default;

        value_A = 0;
        value_B = 0;

        prev_value_A = 0;
        prev_value_B = 0;

        rev = 0;

    }

    void update_knob(uint8_t reading_from_matrix, bool press_reading)
    {
        value_A = (reading_from_matrix & 0b01);
        value_B = ((reading_from_matrix & 0b10) >> 1);
        value_S = press_reading;

  
        rev = 0;

        if((prev_value_B == 0 && prev_value_A == 0 && value_B == 0 && value_A == 1) ||
            (prev_value_B == 1 && prev_value_A == 1 && value_B == 1 && value_A == 0))
        {
          rev = 1;
        }
        else if((prev_value_B == 0 && prev_value_A == 1 && value_B == 0 && value_A == 0) ||
            (prev_value_B == 1 && prev_value_A == 0 && value_B == 1 && value_A == 1))
        {
          rev = -1;
        }
        // detect illegal cases
        else if((prev_value_B == 0 && prev_value_A == 0 && value_B == 1 && value_A == 0) ||
            (prev_value_B == 0 && prev_value_A == 1 && value_B == 1 && value_A == 0) ||
            (prev_value_B == 1 && prev_value_A == 0 && value_B == 0 && value_A == 1) ||
            (prev_value_B == 1 && prev_value_A == 1 && value_B == 0 && value_A == 0))
        {
           rev = last_legal_rev;
        }

        if(value_S == false) knob_control_data = knob_control_data_default;
        knob_control_data += rev;
        limit_knob_control_data();

        prev_value_A = value_A;
        prev_value_B = value_B;
        last_legal_rev = rev;

    }

    int8_t knob_data()
    {
        return knob_control_data;
    }

    void limit_knob_control_data()
    {
        if(knob_control_data > knob_control_data_high_limit)
            knob_control_data = knob_control_data_high_limit;

        if(knob_control_data < knob_control_data_low_limit)
           knob_control_data = knob_control_data_low_limit;
    }


private:

    bool value_A;
    bool value_B;
    bool value_S;
    bool prev_value_A;
    bool prev_value_B;

    uint8_t knob_control_data;
    uint8_t knob_control_data_high_limit;
    uint8_t knob_control_data_low_limit;
    uint8_t knob_control_data_default;


    int8_t rev;
    int8_t last_legal_rev;

};