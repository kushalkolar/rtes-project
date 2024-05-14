#include <mbed.h>
#include <arm_math.h>
#include "drivers/LCD_DISCO_F429ZI.h"

// =================================================
// * Recitation 5: SPI and Gyroscope *
// =================================================

// TODOs:
// [1] Get started with an SPI object instance and connect to the Gyroscope!
// [2] Read the XYZ axis from the Gyroscope and Visualize on the Teleplot. 
// [3] Fetching Data from the sensor via Polling vs Interrupt ?

// Define control register addresses and their configurations
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28
#define lower_threshold 50
#define upper_threshold 2000

EventFlags flags;
LCD_DISCO_F429ZI lcd;

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

bool binary_classifier(float* esd)
{
    float energy_value = esd[1] + esd[2] + esd[3];

    // printf("Energy Value =%f", energy_value);
    if (energy_value > lower_threshold && energy_value < upper_threshold)
    {
        return true;
    }
    else
    {
        return false;
    }

}

// void  printLCD(float* esd)
// {
    // if (binary_classifier(&esd[0])) {
    //     printf("Tremor detected\n");
    //     lcd.Clear(LCD_COLOR_RED);
    //     lcd.SetBackColor(LCD_COLOR_RED);
    //     lcd.SetTextColor(LCD_COLOR_WHITE);
    //     lcd.DisplayStringAtLine(5, (uint8_t *)"Tremor detected");
    // } else {
    //     printf("No tremor detected\n");
    //     lcd.Clear(LCD_COLOR_GREEN);
    //     lcd.SetBackColor(LCD_COLOR_GREEN);
    //     lcd.SetTextColor(LCD_COLOR_BLACK);
    //     lcd.DisplayStringAtLine(5, (uint8_t *)"Monitoring...");
    // }
// }

void energy_spectral_density(int n_samples, float32_t *sample_buffer, float32_t *esd) {
    // computes discrete energy (ESD) spectral density from FFT
    // places ESD output in `psd_output_buffer`
    // see: https://en.wikipedia.org/wiki/Spectral_density#Energy_spectral_density
    // This computers a discrete by basically computing the square of the absolute
    // value of each FFT bin. Basically, each FFT bin is a complex number in the form:
    // a + bi, where i = sqrt(-1). Then ESD of each bin =
    // abs(a + bi) = (sqrt(a^2 + b^2))^2 = a^2 + b^2
    printf("starting esd\n");

    float32_t output_buffer[128];
    float32_t *output_ptr = output_buffer;

    arm_rfft_fast_instance_f32 handler;

    bool flag = false;
    float energy_value;

    arm_status status;
    status = arm_rfft_fast_init_f32(&handler, 128);

    arm_rfft_fast_f32(&handler, sample_buffer, output_ptr, 0);

    // printf("fft: %3.2f\n", output_buffer[0]);

    // apparently the output buffer of ARM FFT is like this:
    // i0_real, i0_complex, i1_real, i1_complex, we we square
    // and add the corresponding real and complex parts

    int i;  // index
    float a;  // real part of FFT bin
    float b;  // complex part of FFT bin
    float value;
    // printf(">esd:0|g,clr\n");
    for (i = 0; i < 127; i+=2) {
        a = output_buffer[i];  // real
        b = output_buffer[i + 1];  // complex
        value = a*a + b*b;
        // printf(">esd: %3.2f|g \n", value);
        esd[i / 2] = value;  // ESD for this bin
    }
    // printLCD(&esd[0]);
    // flag = binary_classifier(&esd[0]);
    energy_value = esd[1] + esd[2] + esd[3];

    printf("Energy Value =%f", energy_value);
    if (energy_value > lower_threshold && energy_value < upper_threshold)
    {
        flag = true;
    }
    else
    {
        flag = false;
    }
    if (flag == 1) {
        printf("Tremor detected\n");
        lcd.Clear(LCD_COLOR_RED);
        lcd.SetBackColor(LCD_COLOR_RED);
        lcd.SetTextColor(LCD_COLOR_WHITE);
        lcd.DisplayStringAt(10,LINE(8),(uint8_t *)"Tremor detected",CENTER_MODE);
    } else {
        printf("No tremor detected\n");
        lcd.Clear(LCD_COLOR_GREEN);
        lcd.SetBackColor(LCD_COLOR_GREEN);
        lcd.SetTextColor(LCD_COLOR_BLACK);
        lcd.DisplayStringAt(10,LINE(8),(uint8_t *)"Monitoring...", CENTER_MODE);
    }
}

void collect_data(int interval, int n_samples, float32_t *sample_buffer) {
    // TODO: See if spi object and flags can be initialized just once in main() and then passed into collect_data
    // Initialize the SPI object with specific pins.
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    // Buffers for sending and receiving data over SPI.
    uint8_t write_buf[32], read_buf[32];

    // Configure SPI format and frequency.
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Configure CTRL_REG1 register.
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Configure CTRL_REG4 register.
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;

    int sample_ix;
    for (sample_ix = 0; sample_ix < 128; sample_ix++) {
        // Prepare to read the gyroscope values starting from OUT_X_L
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        // Perform the SPI transfer to read 6 bytes of data (for x, y, and z axes)
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Convert the received data into 16-bit integers for each axis
        raw_gx = (((int16_t)read_buf[2]) << 8) | ((int16_t) read_buf[1]);
        raw_gy = (((int16_t)read_buf[4]) << 8) | ((int16_t) read_buf[3]);
        raw_gz = (((int16_t)read_buf[6]) << 8) | ((int16_t) read_buf[5]);

        // Print the raw values for debugging 
        // printf("RAW \t\tgx: %d \t gy: %d \t gz: %d \t\n", raw_gx, raw_gy, raw_gz);

        // Convert raw data to actual values using a scaling factor
        gx = ((float) raw_gx) * SCALING_FACTOR;
        gy = ((float) raw_gy) * SCALING_FACTOR;
        gz = ((float) raw_gz) * SCALING_FACTOR;

        sample_buffer[sample_ix] = gy;

        // Print the actual values
        // printf("Actual -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", gx, gy, gz);

        // printf(">x_axis: %5.2f|g \n", gx);
        // printf(">y_axis: %5.2f|g \n", gy);
        // printf(">z_axis: %5.2f|g \n", gz);

        thread_sleep_for(interval);
    }
}


int main()
{
    // collect data every 4ms
    // sampling rate = 1s / interval = 250Hz
    int interval = 4;

    // collect 128 samples for a total of 512ms of data
    int n_samples = 128;

    // a buffer to store the samples, array of size n_samples
    float32_t sample_buffer[128];
    float32_t esd[128];

    lcd.Clear(LCD_COLOR_GREEN);
    lcd.SetBackColor(LCD_COLOR_GREEN);
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.DisplayStringAtLine(5, (uint8_t *)"Monitoring...");

    while (1) {
        collect_data(interval, n_samples, &sample_buffer[0]);
        printf("Data collected!\n");
        energy_spectral_density(n_samples, &sample_buffer[0], &esd[0]);
    }
}
