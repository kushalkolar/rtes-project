#include <mbed.h>
#include <arm_math.h>

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

EventFlags flags;

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)


void energy_spectral_density(float32_t *data, float32_t *esd) {
    // computes discrete energy (ESD) spectral density from FFT
    // places ESD output in `psd_output_buffer`
    // see: https://en.wikipedia.org/wiki/Spectral_density#Energy_spectral_density
    // This computers a discrete by basically computing the square of the absolute
    // value of each FFT bin. Basically, each FFT bin is a complex number in the form:
    // a + bi, where i = sqrt(-1). Then ESD of each bin =
    // abs(a + bi) = (sqrt(a^2 + b^2))^2 = a^2 + b^2

    float32_t output_buffer[1024];
    float32_t *output_ptr = output_buffer;

    arm_rfft_fast_instance_f32 handler;

    arm_rfft_fast_f32(
        &handler,
        data,
        output_ptr,
        0  // do rfft, if 1 does inverse rfft
    );

    // apparently the output buffer of ARM FFT is like this:
    // i0_real, i0_complex, i1_real, i1_complex, we we square
    // and add the corresponding real and complex parts

    int i;  // index
    int a;  // real part of FFT bin
    int b;  // complex part of FFT bin
    for (i = 0; i < 1023; i+=2){
        a = output_buffer[i];  // real
        b = output_buffer[i + 1];  // complex
        esd[i / 2] = a*a + b*b;  // ESD for this bin
    }
}


int main()
{
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

    // collect 256 sample points at 2ms intervals
    // 256 * 2ms = 512 ms of data
    float sample_buffer[256];

    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;

    int sample_ix;
    for (sample_ix = 0; sample_ix < 256; sample_ix ++){
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
        printf("RAW -> \t\tgx: %d \t gy: %d \t gz: %d \t\n", raw_gx, raw_gy, raw_gz);

            printf(">x_axis: %d|g \n", raw_gx);
            printf(">y_axis: %d|g \n", raw_gy);
            printf(">z_axis: %d|g \n", raw_gz);

        // Convert raw data to actual values using a scaling factor
        gx = ((float) raw_gx) * SCALING_FACTOR;
        gy = ((float) raw_gy) * SCALING_FACTOR;
        gz = ((float) raw_gz) * SCALING_FACTOR;

        // Print the actual values
        printf("Actual -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", gx, gy, gz);

        // read every 2ms
        thread_sleep_for(2);
    }
    // after 512ms of data is collected, analyze it and determine if a tremor is present

}
