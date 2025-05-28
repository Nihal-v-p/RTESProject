//RTES Final Project
// Team Members
// Nihal V P nv2331
// Saurabh Dahale sd5726
// Arjav Virani av3451

#include "mbed.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "vector"

#include "drivers/LCD_DISCO_F429ZI.h"

const int SAMPLES = 128;  // Number of samples
const float SAMPLING_FREQUENCY = 100.0; // Sampling Frequency
const float FREQUENCY_RESOLUTION = SAMPLING_FREQUENCY / SAMPLES;

// Bins used for checking the right frequencies of resting trmor
const int LOWER_BIN = floor(3.0 / FREQUENCY_RESOLUTION);
const int UPPER_BIN = ceil(6.0 / FREQUENCY_RESOLUTION);
const float AMPLITUDE_THRESHOLD = 10;
const float AMPLITUDE_DANGER_THRESHOLD = 15;

// FFT instance
// used for fft transform 
arm_rfft_fast_instance_f32 S;

#define WINDOW_SIZE 10 // Moving Avg window size

// Gyroscope's settings
// Updated for 100 Hz and Cut off 12.5
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b00'00'1'1'1'1

//Updated for FS = 245 dps 
#define CTRL_REG4 0x23 // Second configure to set the DPS
#define CTRL_REG4_CONFIG 0b0'0'00'0'00'0

#define CTRL_REG3 0x22 // page 32
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define OUT_X_L 0x28

#define SPI_FLAG 1
#define DATA_READY_FLAG 2


//Scaling factor adjusted according to FS value
#define SCALING_FACTOR (8.75f * 0.017453292519943295769236907684886f / 1000.0f)

#define FILTER_COEFFICIENT 0.1f

// EventFlags object declaration
EventFlags flags;

// spi callback function
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}


// data ready callback function
void data_cb() {
    flags.set(DATA_READY_FLAG);
}

//databuffer for storing input to perform fft
std::vector<float> dataBuffer(SAMPLES);
int dataIndex = 0;


int main() {
    //spi initialization
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    uint8_t write_buf[32], read_buf[32];

    //interrupt initialization
    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);

    LCD_DISCO_F429ZI lcd;
    
    //spi format and frequency
    spi.format(8, 3);
    spi.frequency(1'000'000);
    // Write to control registers --> spi transfer
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[1] = 0xFF;

    //(polling for\setting) data ready flag
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {
        flags.set(DATA_READY_FLAG);
    }

    // Moving Average definitions
    float window_gx[WINDOW_SIZE] = {0};
    float window_gy[WINDOW_SIZE] = {0};
    float window_gz[WINDOW_SIZE] = {0};
    int window_index = 0;

    lcd.Clear(LCD_COLOR_GREEN);

    while (1) {
        int16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;

        flags.wait_all(DATA_READY_FLAG);
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Process raw data
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        gx = ((float)raw_gx) * SCALING_FACTOR;
        gy = ((float)raw_gy) * SCALING_FACTOR;
        gz = ((float)raw_gz) * SCALING_FACTOR;

        window_gx[window_index] = gx;
        window_gy[window_index] = gy;
        window_gz[window_index] = gz;

        // //Compute the moving average
        
        float avg_gx = 0.0f, avg_gy = 0.0f, avg_gz = 0.0f;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            avg_gx += window_gx[i];
            avg_gy += window_gy[i];
            avg_gz += window_gz[i];
        }
        avg_gx /= WINDOW_SIZE;
        avg_gy /= WINDOW_SIZE;
        avg_gz /= WINDOW_SIZE;

        //Increment and wrap the window index

        window_index = (window_index + 1) % WINDOW_SIZE;

        //print will slow down the calculations and is only to be used for teleplot
        printf(">x_axis_mov:%4.5f\n", avg_gx);
        printf(">y_axis_mov:%4.5f\n", avg_gy);
        printf(">z_axis_mov:%4.5f\n", avg_gz);

        // collect samples till Sampling Range
        if (dataIndex<SAMPLES){
            dataBuffer[dataIndex++] = sqrt(avg_gx * avg_gx + avg_gy * avg_gy + avg_gz * avg_gz);
            

        }
        //Once collected perform fft and analysis
        else{
            float32_t input[SAMPLES]; 
            for (int i = 0; i < SAMPLES; ++i) {
                input[ i] = dataBuffer[i];
            }
            float32_t mag[SAMPLES/2+1];
            // Perform FFT
            float32_t output[SAMPLES*2];
            //initialize the fft function
            arm_rfft_fast_init_f32(&S, SAMPLES);
            //perform fft
            arm_rfft_fast_f32(&S, input, output, 0);
            //calculate magnitude
            arm_cmplx_mag_f32(output, mag, SAMPLES/2+1);
  
            float maxAmplitude = 0;
            int maxIndex = 0;

            // Iterate over the frequency bins of interest (3-6 Hz)
            for (int i = LOWER_BIN+1; i <= UPPER_BIN-2; ++i) {
                    //printf("Magnitude is %f at i =%d",mag[i],i);
                     if (mag[i] > maxAmplitude) {
                        maxAmplitude = mag[i];
                        maxIndex = i;
                    }
            }
        
            // Compare the maximum amplitude found to the threshold
            if (maxAmplitude > AMPLITUDE_THRESHOLD) {
                    //float tremorFrequency = maxIndex * FREQUENCY_RESOLUTION;
                    //printf("Tremor detected at frequency: %.2f Hz with amplitude: %.2f\n", tremorFrequency, maxAmplitude);
                    //Set Background to Red if Intensity is High in Resting Tremor Freq Range 
                    if(maxAmplitude > AMPLITUDE_DANGER_THRESHOLD)
                        lcd.Clear(LCD_COLOR_RED);
                    else 
                    //Set Background to Yellow  if Intensity is Mild in Resting Tremor Freq Range 
                        lcd.Clear(LCD_COLOR_YELLOW);
                    lcd.SetTextColor(LCD_COLOR_WHITE);
                    lcd.SetBackColor(LCD_COLOR_BLACK);

                    lcd.DisplayStringAt(0,LINE(4),(uint8_t *)"RTES Project",CENTER_MODE);

                    //Display Tremor Detected
                    lcd.DisplayStringAt(0,LINE(6),(uint8_t *)"Tremor Detected !!!",CENTER_MODE);

                    // Display Intensity as Magnitude Value 
                    char buf[60];
                    snprintf(buf,60,"Magnitude - %.2f",maxAmplitude);
                    lcd.DisplayStringAt(0,LINE(7),(uint8_t *)buf,CENTER_MODE);

            } else {
                    //printf("No tremor detected.\n");
                    //Set Background to Green if no Tremor in Freq Range
                    lcd.Clear(LCD_COLOR_GREEN);
            }
            dataIndex=0;
        }


        thread_sleep_for(100);
    }
}