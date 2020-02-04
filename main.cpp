#include "mbed.h"
#include "math.h"

/**
 * All the following code is original, no libraries, other than what Mbed 
 * provides, were used.
 */

I2C i2c(p9, p10);
Serial lcd_serial(p13, p14);

InterruptIn lidar_button(p21);
InterruptIn air_button(p22);

/*
Byte reference:

index: 7   6  5  4  | 3  2  1  0
value: 128 64 32 16 | 8  4  2  1
       8   4  2  1

I2C reference:

Addresses are shifted to the left once b/c the LSB in an I2C address
is used to indicate if the transaction is a read or write. This bit
will be set by the Mbed I2C API.
*/

/**
 * Button constants
 */
const int SENSOR_MODE_AIR = 0;
const int SENSOR_MODE_LIDAR = 1;

// determines which sensor is being read and displayed
int sensor_mode = SENSOR_MODE_AIR;

/**
 * LiDAR constants
 */
const int LIDAR_ADDR = 0x62 << 1;

const char LIDAR_ACQ_CMD_REG = 0x00;
const char LIDAR_ACQ_CMD = 0x04;

const char LIDAR_STATUS_REG = 0x01;
const char LIDAR_STATUS_BUSY_MASK = 0x01;
const char LIDAR_STATUS_HEALTH_MASK = 0x20;

const char LIDAR_DELTA_VELOCITY_REG = 0x09;

// The high byte of the value is stored in 0x0f and the low byte in 0x10.
// The LiDAR sensor can automatically increment register read addresses if the
// MSB of an address to be set 1. 
// Setting the MSB of 0x0f to 1 = 0x8f, which triggers a read of first 0x0f then
// 0x10.
const char LIDAR_DIST_REG = 0x8f;

/**
 * Air sensor constants
 */
const int AIR_ADDR = 0x5A << 1;

const char AIR_STATUS_REG = 0x00;
const char AIR_STATUS_ERROR_MASK = 0x01;
const char AIR_STATUS_DATA_READY_MASK = 0x08;
const char AIR_STATUS_APP_VALID_MASK = 0x16;
const char AIR_STATUS_FW_MODE_MASK = 0x80;
const char AIR_STATUS_FW_MODE_BOOT = 0;
const char AIR_STATUS_FW_MODE_APP = 1;

const char AIR_MODE_REG = 0x01;
const char AIR_MODE_DRIVE_MODE_MASK = 0x70;
const char AIR_MODE_IDLE = 0x00;
const char AIR_MODE_1_SECOND = 0x01;

const char AIR_ERROR_ID_REG = 0xE0;
const char AIR_ERROR_ID_BAD_WRITE = 0x00;
const char AIR_ERROR_ID_BAD_READ = 0x01;
const char AIR_ERROR_ID_BAD_MODE = 0x02;
const char AIR_ERROR_ID_MAX_RESISTANCE = 0x03;
const char AIR_ERROR_ID_HEATER_FAULT = 0x04;
const char AIR_ERROR_ID_HEATER_SUPPLY = 0x05;

const char AIR_ALG_RESULT_DATA_REG = 0x02;

const char AIR_BOOT_APP_START_REG = 0xF4;

const uint16_t AIR_TVOC_MAX = 1187;

/**
 * LCD constants
 */
const char LCD_CTRL_START_CHAR = 0xFE;
const char LCD_RESET_CHAR = 0x01;

void die(char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    
    vprintf(fmt, args);
    printf("\r\n");
    
    va_end(args);
    
    exit(1);
}

/**
 * LiDAR status.
 */
typedef struct {
    /**
     * Indicates if the device is busy.
     * Boolean.
     */
    char busy;
    
    /**
     * Indicates if the device is healthy.
     * Boolean.
     */
    char healthy;
} lidar_status_t;

/**
 * Reads the LiDAR sensor's status.
 */
void lidar_read_status(lidar_status_t *lidar_status) {
    // Read register
    if (i2c.write(LIDAR_ADDR, &LIDAR_STATUS_REG, 1) != 0) {
        die("lidar: read_status: failed to select status register for read");
    }
    
    char buf;
    if (i2c.read(LIDAR_ADDR, &buf, 1) != 0) {
        die("lidar: read_status: failed to read status register");
    }

    // Unpack into lidar_status_t
    lidar_status->busy = buf & LIDAR_STATUS_BUSY_MASK;
    lidar_status->healthy = (buf * LIDAR_STATUS_HEALTH_MASK) >> 5;
}

/**
 * Exits program if the LiDAR sensor is in an error state.
 */
void lidar_die() {
    lidar_status_t lidar_status;
    lidar_read_status(&lidar_status);
    
    if (!lidar_status.healthy) {
        die("lidar: sensor is not healthy");
    }
}

/**
 * Sends an acquire data command to the LiDAR sensor.
 */
void lidar_write_acq_cmd() {
    char buf[2] = {
        LIDAR_ACQ_CMD_REG, 
        LIDAR_ACQ_CMD
    };
    if (i2c.write(LIDAR_ADDR, buf, 2) != 0) {
        die("lidar: write_acq_cmd: failed to write acquire command");
    }
}

/**
 * Reads the distance value from the LiDAR sensor.
 * Returns: Distance in centimeters.
 *
 */
uint16_t lidar_read_distance() {
    // Read
    if (i2c.write(LIDAR_ADDR, &LIDAR_DIST_REG, 1) != 0) {
        die("lidar: read_distance: failed to select distance registers for read");
    }
    
    char buf[2];
    if (i2c.read(LIDAR_ADDR, buf, 2) != 0) {
        die("lidar: read_distance: failed to read distance registers");
    }
    
    // Re-assemble
    return buf[1] | (buf[0] << 8);
}

/**
 * Reads the delta velocity value from the LiDAR sensor.
 * Returns: Difference in last velocity reading in cm.
 */
int8_t lidar_read_delta_velocity() {
    if (i2c.write(LIDAR_ADDR, &LIDAR_DELTA_VELOCITY_REG, 1) != 0) {
        die("lidar: read_delta_velocity: failed to select delta velocity register for read");
    }
    
    char buf;
    if (i2c.read(LIDAR_ADDR, &buf, 1) != 0) {
        die("lidar: read_delta_velocity: failed to read delta velocity register");
    }
    
    return (int8_t)buf;
}

/**
 * Air sensor status register fields.
 */
typedef struct {
    /**
     * Firmware mode, see AIR_STATUS_FW_MODE_* constants.
     */
    char fw_mode;
    
    /**
     * Indicates if the application on the sensor is valid.
     * Boolean.
     */
    char app_valid;
    
    /**
     * If an error has occured.
     * Boolean.
     */
    char error;
    
    /**
     * If a new data measurement is available.
     * Boolean.
     */
    char data_ready;
    
    /**
     * Raw bit packed status valid, useful to have here for debugging purposes.
     */
     char raw;
} air_status_t;

/**
 * Read air sensor status register into the air_status arugment.
 */
void air_read_status(air_status_t *air_status) {
    if (i2c.write(AIR_ADDR, &AIR_STATUS_REG, 1) != 0) {
        die("air: read_status: failed to select status register");
    }
    
    char raw_status;
    if (i2c.read(AIR_ADDR, &raw_status, 1) != 0) {
        die("air: read_status: failed to read air status register");
    }
    
    air_status->fw_mode = (raw_status & AIR_STATUS_FW_MODE_MASK) >> 7;
    air_status->app_valid = (raw_status & AIR_STATUS_APP_VALID_MASK) >> 4;
    air_status->data_ready = (raw_status &AIR_STATUS_DATA_READY_MASK) >> 3;
    air_status->error = raw_status & AIR_STATUS_ERROR_MASK;
    air_status->raw = raw_status;
}

/**
 * Read error ID from the air sensor.
 * Returns: Error ID
 */
char air_read_error_id() {
    if (i2c.write(AIR_ADDR, &AIR_ERROR_ID_REG, 1) != 0) {
        die("air: read_error_id: failed to select error id register");
    }
    
    char air_error_id;
    if (i2c.read(AIR_ADDR, &air_error_id, 1) != 0) {
        die("air: read_error_id: failed to read error id");
    }
    
    return air_error_id;
}

/**
 * Exits program with there is an error with the air sensor.
 */
void air_die() {
    air_status_t air_status;
    air_read_status(&air_status);
    if (air_status.error) {
        char air_error_id = air_read_error_id();
        char *str_air_error_id = NULL;
        
        switch(air_error_id) {
            case AIR_ERROR_ID_BAD_WRITE:
                str_air_error_id = "a write occurred for an invalid register address";
                break;
            case AIR_ERROR_ID_BAD_READ:
                str_air_error_id = "a read occurred for an invalid register address";
                break;
            case AIR_ERROR_ID_BAD_MODE:
                str_air_error_id = "the measurement drive mode is invalid";
                break;
            case AIR_ERROR_ID_MAX_RESISTANCE:
                str_air_error_id = "the resistance is set too high";
                break;
            case AIR_ERROR_ID_HEATER_FAULT:
                str_air_error_id = "the heater's current was not in range";
                break;
            case AIR_ERROR_ID_HEATER_SUPPLY:
                str_air_error_id = "the heater's voltage is not being applied correctly";
                break;
            default:
                str_air_error_id = "unknown error!";
                break;
        }
        
        die("air: error: %s\r\n", str_air_error_id);
    }
}

/**
 * Boot air sensor.
 * If already booted exits silently.
 */
void air_boot() {
    // Check if sensor is in a valid state to be booted
    air_status_t air_status;
    air_read_status(&air_status);
    
    // Check if already booted
    if(air_status.fw_mode == AIR_STATUS_FW_MODE_APP) {
        // Already booted
        return;
    }
    
    // Check if a valid application is loaded to be booted
    if (!air_status.app_valid) {
        die("air: boot: cannot boot, invalid app on device");
    }
    
    // Send boot command
    if (i2c.write(AIR_ADDR, &AIR_BOOT_APP_START_REG, 1) != 0) {
        die("air: boot: failed to boot");
    }
}

/**
 * Read measurement drive mode.
 * Returns: drive mode
 */
char air_read_mode() {
    if (i2c.write(AIR_ADDR, &AIR_MODE_REG, 1) != 0) {
         die("air: read_mode: failed to set drive mode to constant high freq");
    }
    
    char air_mode_buf;
    if (i2c.read(AIR_ADDR, &air_mode_buf, 1) != 0) {
        die("air: read_mode: failed to read mode register");
    }
    
    air_mode_buf = (air_mode_buf & AIR_MODE_DRIVE_MODE_MASK) >> 4;
    
    return air_mode_buf;
}

/**
 * Set measurement drive mode.
 * Previous measurement drive mode is read so new drive_mode value can be inserted into the
 * bitpacked register correctly.
 */
void air_write_mode(char drive_mode) {
    // Read current measurement mode
    if (i2c.write(AIR_ADDR, &AIR_MODE_REG, 1) != 0) {
        die("air: write_mode: failed to select measurement mode register");
    }
    
    char measurement_mode;
    if (i2c.read(AIR_ADDR, &measurement_mode, 1) != 0) {
        die("air: write_mode: failed to read measurement mode register");
    }
    
    // Bitpack new drive_mode into measurement_mode
    char write_drive_mode = drive_mode << 4;
    measurement_mode = measurement_mode & (!AIR_MODE_DRIVE_MODE_MASK);
    
    char write_measurement_mode = write_drive_mode | measurement_mode;
    
    char buf[2] = {
        AIR_MODE_REG,
        write_measurement_mode,
    };
    
    if (i2c.write(AIR_ADDR, buf, 2) != 0) {
        die("air: write_mode: failed to write mode %d", drive_mode);
    }
}

/**
 * Air sensor algorithm result data.
 */
typedef struct {
    /**
     * Equivalent calculated carbon-dioxide (eCO2) in ppm from 400 to 8192.
     */
    uint16_t eco2;
    
    /**
     * Total volume of carbon (TVOC) in ppb from 0 1187.
     */
    uint16_t tvoc;
} air_alg_result_t;

void air_read_alg_result(air_alg_result_t *air_alg_result) {
    // Read register
    if (i2c.write(AIR_ADDR, &AIR_ALG_RESULT_DATA_REG, 1) != 0) {
        die("air: read_alg_result: failed to select alg result data register");
    }
    
    char buf[4];
    if (i2c.read(AIR_ADDR, buf, 4) != 0) {
        die("air: read_alg_result: failed to read alg result data register");
    }
    
    // Unpack into air_alg_result_data_t
    air_alg_result->eco2 = (buf[0] << 8) | buf[1];
    air_alg_result->tvoc = (buf[2] << 8) | buf[3];
}
    
/**
 * Displays text on the LCD, same arugments as printf.
 */
void lcd_printf(char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    
    lcd_serial.vprintf(fmt, args);
    
    va_end(args);
}

/**
 * Clears all text and moves the cursor back to the first line on the LCD.
 */
void lcd_reset() {
    lcd_serial.putc(LCD_CTRL_START_CHAR);
    lcd_serial.putc(LCD_RESET_CHAR);
}

void lidar_button_pressed() {
    sensor_mode = SENSOR_MODE_LIDAR;
}

void air_button_pressed() {
    sensor_mode = SENSOR_MODE_AIR;
}

int main() {
    int8_t lidar_velocity = 0;
    air_status_t air_status;
    
    // Print header
    printf(" ----------\r\n");
    printf("| TRICODER |\r\n");
    printf(" ----------\r\n");
    
    // Setup mode switching buttons
    lidar_button.fall(callback(lidar_button_pressed));
    air_button.fall(callback(air_button_pressed));
    
    // Boot air sensor
    printf("air: booting\r\n");
    air_boot();
    air_die();
    printf("air: booted\r\n");
    
    // Set measurement drive mode
    printf("air: setting measurement mode\r\n");
    air_write_mode(AIR_MODE_1_SECOND);
    air_die();
    printf("air: set measurement mode\r\n");
    
    while (1) {
        if (sensor_mode == SENSOR_MODE_LIDAR) {
            // Send acquire command
            lidar_write_acq_cmd();
            
            // Wait until lidar isn't busy
            lidar_status_t lidar_status;
            
            do {
                lidar_read_status(&lidar_status);
                wait(0.5);
            } while (lidar_status.busy);
        
            // Read distance
            uint16_t lidar_distance = lidar_read_distance();
            
            if (lidar_distance >= ((uint16_t)9999)) {
                lidar_distance = 9999;
            }
            
            // Read velocity
            lidar_velocity += lidar_read_delta_velocity();
            
            // Display measurements on LCD
            lcd_reset();
            lcd_printf("distance %d cm", lidar_distance);
            
            int lidar_distance_len = 1;
            
            if (lidar_distance >= ((uint16_t)1000)) {
                lidar_distance_len = 4;
            } else if (lidar_distance >= ((uint16_t)100)) {
                lidar_distance_len = 3;
            } else if (lidar_distance >= ((uint16_t)10)) {
                lidar_distance_len = 2;
            }
            
            for (int i = 0; i < (4-lidar_distance_len); i++) {
                lcd_printf(" ");
            }
            
            lcd_printf("vel. %d cm/s", lidar_velocity);
            
            printf("lidar: distance=%d cm, velocity=%d cm/s\r\n", lidar_distance,
                lidar_velocity);
        } else {
            // Poll until new sample is ready
            air_read_status(&air_status);
            
            do {
                // Read status
                printf("air: polling status until data ready\r\n");
                air_read_status(&air_status);
                air_die();
                wait(0.5);
            } while (!air_status.data_ready);
        
            printf("air: data ready\r\n");
            
            air_alg_result_t air_alg_result;
            air_read_alg_result(&air_alg_result);
            
            printf("air: tvoc=%d\r\n", air_alg_result.tvoc);
            
            // Display on LCD
            lcd_reset();
            lcd_printf("0 CO2 (ppm) 1100");
            
            float tvoc_percent = ((float)air_alg_result.tvoc) / ((float)AIR_TVOC_MAX);
            if (tvoc_percent > 1) {
                tvoc_percent = 1;
            }
            int tvoc_bars = (int)floor(tvoc_percent * ((float)14));
            
            lcd_printf("I");
            for (int i = 0; i < tvoc_bars; i++) {
                lcd_printf("=");
            }
            for (int i = 0; i < 14-tvoc_bars; i++) {
                lcd_printf(" ");
            }
            lcd_printf("I");
        }
    }
}
