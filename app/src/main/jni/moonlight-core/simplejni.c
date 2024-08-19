#include <Limelight.h>

#include <jni.h>
#include <android/log.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <string.h>
#include <pthread.h>

#include "minisdl.h"
#include "controller_type.h"
#include "controller_list.h"
#include <string.h>
#include <unistd.h>   // For usleep
#include <stdlib.h>   // For rand and srand

#include <math.h>
#include <libusb.h>
#include <time.h>


#define TRANSMIT_PAYLOAD 0x04
#define ENTER_SNIFFER_MODE 0x05
#define ENTER_PROMISCUOUS_MODE 0x06
#define ENTER_TONE_TEST_MODE 0x07
#define TRANSMIT_ACK_PAYLOAD 0x08
#define SET_CHANNEL 0x09
#define GET_CHANNEL 0x0A
#define ENABLE_LNA_PA 0x0B
#define TRANSMIT_PAYLOAD_GENERIC 0x0C
#define ENTER_PROMISCUOUS_MODE_GENERIC 0x0D
#define RECEIVE_PAYLOAD 0x12
//#define TRANSMIT_PAYLOAD_MOUSE 0x13
#define EP_DATA_IN	0x01
#define EP_DATA_OUT	0x81
#define M_PI 3.14159265358979323846
#define RADTODEG 180 / M_PI
#define DEGTORAD M_PI / 180

bool movingmouse = false;
int currentChannel = 0;
int dwellTime = 5;
int usbTimeout = 50;
unsigned char keepalive[] = { TRANSMIT_PAYLOAD,5, 0, 0, 0x00, 0x40, 0x00, 0x21, 0x9F }; //00:40:00:21:9F
unsigned char address[] = { 0xE7, 0xF1, 0x97, 0xE4, 0x09 }; //E7:F1:97:E4:09 00:40:00:21:9F
libusb_device_handle *devh;
libusb_device_handle *devhMouse;

bool mouse1 = false;
bool mouse2 = false;
bool mouse3 = false;
bool mouse4 = false;
bool mouse5 = false;
int scroll_v = 0;

void enableLNA()
{
    int actual_length = 0;
    unsigned char data[] = { ENABLE_LNA_PA, 0 };
    unsigned char buff[64];
    int rc = libusb_bulk_transfer(devh, EP_DATA_IN, data, (int)(64), &actual_length, usbTimeout);
    if (rc < 0 || actual_length != 64) {
        //fprintf(stderr, "[+] Error enabling LNA: %s\n", libusb_error_name(rc));
    }
    rc = libusb_bulk_transfer(devh, EP_DATA_OUT, buff, (int)((64)), &actual_length, usbTimeout);
}

void setChecksum(uint8_t* payload, uint8_t len)
{
    uint8_t checksum = 0;

    for (uint8_t i = 0; i < (len - 1); i++)
        checksum += payload[i];

    payload[len - 1] = -checksum;
}

void constructPacket(unsigned char *readyPack, const unsigned char *testPackge3) {
    static const unsigned char header[] = { TRANSMIT_PAYLOAD, 10, 0, 0 };

    // Copy the header into readyPack
    memcpy(readyPack, header, sizeof(header));

    // Copy the testPackge3 into readyPack
    memcpy(readyPack + 4, testPackge3, 10);  // assuming testPackge3 is always 10 bytes
}

void move(uint8_t *package, int16_t x_move, int16_t y_move, uint8_t scroll_v, uint8_t scroll_h, bool leftClick, bool rightClick, bool mouse3)
{
    uint8_t mouse_payload[] = { 0x00, 0xC2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    uint32_t cursor_velocity;

    cursor_velocity = ((uint32_t)y_move & 0xFFF) << 12 | (x_move & 0xFFF);

    memcpy(mouse_payload + 4, &cursor_velocity, 3);

    if (leftClick)
    mouse_payload[2] = 1;

    if (rightClick)
    mouse_payload[2] |= 1 << 1;

    if (mouse3)
    mouse_payload[2] |= 1 << 2;

    mouse_payload[7] = scroll_v;
    mouse_payload[8] = scroll_h;

    setChecksum(mouse_payload, 10);

    for (size_t i = 0; i < 10; i++)
    {
    package[i] = mouse_payload[i];
    }
}

char sendPing()
{
    int actual_length = 0;
    unsigned char data[] = { TRANSMIT_PAYLOAD, 4, 15, 15, 0x0f, 0x0f, 0x0f, 0x0f };

    int rc = libusb_bulk_transfer(devh, EP_DATA_IN, data, (int)(64), &actual_length, 2500);
    if (rc < 0 || actual_length != 64) {
        //fprintf(stderr, "[-] Error transfering payload ping: %s\n", libusb_error_name(rc));
    }
    actual_length = 0;
    unsigned char buff[64];
    rc = libusb_bulk_transfer(devh, EP_DATA_OUT, buff, (int)((64)), &actual_length, 2500);
    if (rc < 0) {
        //fprintf(stderr, "[-] Error reading resp ping: %s\n", libusb_error_name(rc));
    }
    return buff[0];
}

bool sendPacket(unsigned char* packet)
{
    int actual_length = 0;
    int rc = libusb_bulk_transfer(devh, EP_DATA_IN, packet, 64, &actual_length, 5);
    if (rc < 0 || actual_length != 64) {
        //fprintf(stderr, "[-] Error transferring payload: %s\n", libusb_error_name(rc));
        return false;
    }
    return true;
}

void setChannel(int channel)
{
    int actual_length = 0;
    unsigned char data[] = { SET_CHANNEL, channel };
    unsigned char buff[64];
    int rc = libusb_bulk_transfer(devh, EP_DATA_IN, data, (int)(64), &actual_length, usbTimeout);
    if (rc < 0 || actual_length != 64) {
        //fprintf(stderr, "[-] Error setting channel: %s\n", libusb_error_name(rc));
    }
    rc = libusb_bulk_transfer(devh, EP_DATA_OUT, buff, (int)((64)), &actual_length, usbTimeout);

}

void sleep_for_random_time() {
    // Seed the random number generator
    srand(time(NULL));

    // Generate a random number between 35 and 79 milliseconds
    int sleepTime = (rand() % 45) + 35;

    // usleep takes microseconds, so multiply by 1000 to convert milliseconds to microseconds
    usleep(sleepTime * 1000);
}

void click(){
    movingmouse = true;
    unsigned char package_click[] = { 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0 };
    unsigned char package_unclick[] = { 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0 };

    unsigned char ready_click[] = { TRANSMIT_PAYLOAD, 10, 0, 0, 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0 };
    unsigned char ready_unclick[] = { TRANSMIT_PAYLOAD, 10, 0, 0, 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0 };

    move(package_click, 0, 0, 0, 0, true, false, false);
    constructPacket(ready_click, package_click);
    sendPacket(ready_click);
    movingmouse = false;

    sleep_for_random_time();
    //put sleep here
    movingmouse = true;

    move(package_unclick, 0, 0, 0, 0, false, false, false);
    constructPacket(ready_unclick, package_unclick);
    sendPacket(ready_unclick);
    movingmouse = false;
}

void moveMouse(int x, int y)
{
    movingmouse = true;
    //printf("x: %d y: %d  \n", x, y);
    unsigned char moveMouse[] = { 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0 };
    unsigned char moveMouse_ready[] = { TRANSMIT_PAYLOAD, 10, 0, 0, 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0 };


    //unsigned char readyPack1[] = { TRANSMIT_PAYLOAD_MOUSE, 10, 0, 0, x ,y ,utilities::scroll,0x0,utilities::usedLeft,utilities::usedRight,0 };

    //sendPacket(readyPack1);

    move(moveMouse, x, y, scroll_v, 0, mouse1, mouse2, mouse3);
    //calcBoth(testPackge3, mstroke.x, mstroke.y, state, scroll);
    constructPacket(moveMouse_ready, moveMouse);
    sendPacket(moveMouse_ready);
    movingmouse = false;
}
#define NUM_CALLS 50
#define TIME_INTERVAL_SEC 1  // Interval duration in seconds

void *thread_functionMouse(void *arg) {
    intptr_t fileDescriptor = (intptr_t)arg;
    __android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "Mouse initialized");

    libusb_context * ctx;
    int r = 0;
    r = libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
    r = libusb_init(&ctx);
    libusb_wrap_sys_device(NULL, fileDescriptor, &devhMouse);

    if(libusb_kernel_driver_active(devhMouse, 0) == 1) {
        r = libusb_detach_kernel_driver(devhMouse, 0);
        if(r < 0) {
            __android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "Failed to detach kernel\n");

        }
    }
    libusb_claim_interface(devhMouse,0);
    r = libusb_claim_interface(devhMouse, 0);
    if(r < 0) {
        __android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "Failed to claim interface\n");
        libusb_close(devhMouse);
        libusb_exit(ctx);
        return NULL;
    }
    unsigned char data[64];
    int actual_length;

    struct timespec t_end, last_ping;
    clock_gettime(CLOCK_MONOTONIC, &t_end);
    clock_gettime(CLOCK_MONOTONIC, &last_ping);
    int call_count = 0;

    while(true) {
        r = libusb_bulk_transfer(devhMouse, 0x81, data, sizeof(data), &actual_length, 0);
        if (r == 0)
        {

            call_count++;

            clock_gettime(CLOCK_MONOTONIC, &t_end);
            long long elapsed_ms = (t_end.tv_sec - last_ping.tv_sec) * 1000 +
                                   (t_end.tv_nsec - last_ping.tv_nsec) / 1000000;

            if (elapsed_ms >= TIME_INTERVAL_SEC * 1000) {  // Check if 1 second has passed
                double frequency = (double)call_count / TIME_INTERVAL_SEC;
                __android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "Frequency: %f Hz", frequency);

                // Reset for the next interval
                call_count = 0;
                clock_gettime(CLOCK_MONOTONIC, &last_ping);
            }
            if (currentChannel == 2 || currentChannel == 4)
            {
                movingmouse = true;
                int x = (int)data[2] - (int)data[3];
                if ((int)data[3] == 255)
                    x += -1;
                int y = (int)data[4] - (int)data[5];
                if ((int)data[5] == 255)
                    y += -1;

                mouse1 = false;
                mouse2 = false;
                mouse3 = false;
                mouse4 = false;
                mouse5 = false;

                scroll_v = 0;
                if (data[0] > 0)
                {
                    if (data[0] > 0)
                    {
                        switch (data[0])
                        {
                            case 1:
                            {
                                mouse1 = true;
                            } break;
                            case 2:
                            {
                                mouse2 = true;
                            } break;
                            case 3:
                            {
                                mouse1 = true;
                                mouse2 = true;
                            } break;
                            case 4:
                            {
                                mouse3 = true;
                            } break;
                            case 5:
                            {
                                mouse3 = true;
                                mouse1 = true;
                            } break;
                            case 6:
                            {
                                mouse3 = true;
                                mouse2 = true;
                            } break;
                            case 7:
                            {
                                mouse3 = true;
                                mouse2 = true;
                                mouse1 = true;
                            } break;
                            case 8:
                            {
                                mouse4 = true;
                            } break;
                            case 9:
                            {
                                mouse4 = true;
                                mouse1 = true;
                            } break;
                            case 10:
                            {
                                mouse4 = true;
                                mouse2 = true;
                            } break;
                            case 11:
                            {
                                mouse4 = true;
                                mouse2 = true;
                                mouse1 = true;
                            } break;
                            case 12:
                            {
                                mouse4 = true;
                                mouse3 = true;
                            } break;
                            case 13:
                            {
                                mouse4 = true;
                                mouse3 = true;
                                mouse1 = true;
                            } break;
                            case 14:
                            {
                                mouse4 = true;
                                mouse3 = true;
                                mouse2 = true;
                            } break;
                            case 15:
                            {
                                mouse4 = true;
                                mouse3 = true;
                                mouse2 = true;
                                mouse1 = true;
                            } break;
                            case 16:
                            {
                                mouse5 = true;
                            } break;
                            case 17:
                            {
                                mouse5 = true;
                                mouse1 = true;
                            } break;
                            case 18:
                            {
                                mouse5 = true;
                                mouse2 = true;
                            } break;
                            case 19:
                            {
                                mouse5 = true;
                                mouse2 = true;
                                mouse1 = true;
                            } break;
                            case 20:
                            {
                                mouse5 = true;
                                mouse3 = true;
                            } break;
                            case 21:
                            {
                                mouse5 = true;
                                mouse3 = true;
                                mouse1 = true;
                            } break;
                            case 22:
                            {
                                mouse5 = true;
                                mouse3 = true;
                                mouse2 = true;
                            } break;
                            case 23:
                            {
                                mouse5 = true;
                                mouse3 = true;
                                mouse2 = true;
                                mouse1 = true;
                            } break;
                            case 24:
                            {
                                mouse5 = true;
                                mouse4 = true;
                            } break;
                            case 25:
                            {
                                mouse5 = true;
                                mouse4 = true;
                                mouse1 = true;
                            } break;
                            case 26:
                            {
                                mouse5 = true;
                                mouse4 = true;
                                mouse2 = true;
                            } break;
                            case 27:
                            {
                                mouse5 = true;
                                mouse4 = true;
                                mouse2 = true;
                                mouse1 = true;
                            } break;
                            case 28:
                            {
                                mouse5 = true;
                                mouse4 = true;
                                mouse3 = true;
                            } break;
                            case 29:
                            {
                                mouse5 = true;
                                mouse4 = true;
                                mouse3 = true;
                                mouse1 = true;
                            } break;
                            case 30:
                            {
                                mouse5 = true;
                                mouse4 = true;
                                mouse3 = true;
                                mouse2 = true;
                            } break;
                            case 31:
                            {
                                mouse5 = true;
                                mouse4 = true;
                                mouse3 = true;
                                mouse2 = true;
                                mouse1 = true;
                            } break;
                            default:
                            {
                                // Handle any case not covered by the switch
                            } break;
                        }
                    }
                }

                if (data[6] != 0)
                {
                    if (data[6] == 255)
                    {
                        scroll_v = -1;
                    }
                    else if (data[6] == 1)
                    {
                        scroll_v = 1;
                    }
                }
                //__android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "sending Mouse movement: X: %d, Y: %d \n",  x, y);
                //__android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "data[5] %d", data[6]);
                //__android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "mouse data %d %d %d %d %d %d %d % d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d /n", data[0], data[1], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23], data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31], data[32], data[33], data[34], data[35], data[36], data[37], data[38], data[39], data[40], data[41], data[42], data[43], data[44], data[45], data[46], data[47], data[48], data[49], data[50], data[51], data[52], data[53], data[54], data[55], data[56], data[57], data[58], data[59], data[60], data[61], data[62], data[63]);
                //if (data[])
                unsigned char testPackge3[] = { 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0 };
                unsigned char readyPack[] = { TRANSMIT_PAYLOAD, 10, 0, 0, 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0 };
                move(testPackge3, x, y, scroll_v, 0, mouse1, mouse2, mouse3);
                constructPacket(readyPack, testPackge3);
                sendPacket(readyPack);
                movingmouse = false;
            }

        }


    }

    //printf("Thread is running with number: %d\n", *number);
    //sleep(2);  // Simulate some work
    // printf("Thread is finishing\n");
    return NULL;
}
bool isZoomed = false;
int full360 = 11363;

void SetIsZoomed() { // CALL THIS EVERY FRAME
    isZoomed = mouse2;
}

int Full360() {
    return isZoomed ? full360 : (full360 * 8 / 10);
}

int GetCoordsX(int delta, int total) {

    double lookAt = delta * 2.0 / total;
    double degrees = atan(lookAt * tan((isZoomed ? 41.5 : 52.0) * DEGTORAD)) * RADTODEG;
    return (Full360() * degrees) / 360;
}

int GetCoordsY(int delta, int total) {

    double lookAt = delta * 2.0 / total;
    double degrees = atan(lookAt * tan((isZoomed ? 26.5 : 36) * DEGTORAD)) * RADTODEG;
    return (Full360() * degrees) / 360;
}


void *thread_function(void *arg) {
    intptr_t fileDescriptor = (intptr_t)arg;
    int startChannel = 2;
    __android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "Initiating libusb device");

    libusb_context * ctx;
    int r = 0;
    r = libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
    r = libusb_init(&ctx);
    libusb_wrap_sys_device(NULL, fileDescriptor, &devh);

    int actual_length = 0;
    unsigned char data[] = { ENTER_SNIFFER_MODE, 5, address[4], address[3], address[2], address[1], address[0] };
    unsigned char buff[64];
    int rc = libusb_bulk_transfer(devh, EP_DATA_IN, data, (int)(64), &actual_length, usbTimeout);
    if (rc < 0 || actual_length != 64) {
        //fprintf(stderr, "[-] Error setting channel: %s\n", libusb_error_name(rc));
    }
    rc = libusb_bulk_transfer(devh, EP_DATA_OUT, buff, (int)((64)), &actual_length, usbTimeout);
    enableLNA();
    setChannel(startChannel);
    currentChannel = startChannel;
    struct timespec t_end, last_ping;
    clock_gettime(CLOCK_MONOTONIC, &t_end);
    clock_gettime(CLOCK_MONOTONIC, &last_ping);
    while(true)
    {
        clock_gettime(CLOCK_MONOTONIC, &t_end);
        long long elapsed_ms = (t_end.tv_sec - last_ping.tv_sec) * 1000 +
                               (t_end.tv_nsec - last_ping.tv_nsec) / 1000000;
        if (elapsed_ms >= dwellTime)
        {
            bool test = sendPing();
            if (test == false)
            {
                dwellTime = 5;
                clock_gettime(CLOCK_MONOTONIC, &last_ping);
                __android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "[-] Ping failed on channel: %d\n", currentChannel);

                if ((currentChannel + 1) > 2 && (currentChannel + 1) < 85)
                {
                    currentChannel++;
                    setChannel(currentChannel);
                }
                else {
                    currentChannel = 2;
                    setChannel(currentChannel);
                }
            }
            else {
                //__android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "[+] Ping success on channel: %d\n", currentChannel);

                //printf("[+] Ping success on channel: %d\n", utilities::currentChannel);
                if (currentChannel == 2 || currentChannel == 4)
                {
                    if (!movingmouse)
                    {
                        //__android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "sending keepalive");
                        sendPacket(keepalive);
                    }

                    dwellTime = 15000;
                }
                clock_gettime(CLOCK_MONOTONIC, &last_ping);

            }
        }
        else{
            if (currentChannel == 2 || currentChannel == 4)
            {
                if (!movingmouse)
                {
                    //__android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "sending keepalive");
                    sendPacket(keepalive);
                    usleep(1000);
                }
            }
        }
        usleep(1000);
    }
    //printf("Thread is running with number: %d\n", *number);
    //sleep(2);  // Simulate some work
   // printf("Thread is finishing\n");
    return NULL;
}

void moverestricted(double xi, double yi, int Z) {
    double x = xi;
    double y = yi;
    double rx = 0;
    double ry = 0;

    while (fabs(x) > Z || fabs(y) > Z) {
        if (abs(Z) < 3) { // Decides random
            int v1 = rand() % (4 - Z) + 2;
            if (v1 == 1) {
                rx = (rand() % 3) - 1;
                ry = (rand() % 3) - 1;
            }
            if (y == 0 && ry == 0) {
                ry = (rand() % 3) - 1;
            }
        } else if (abs(Z) < 5) { // Decides random
            int v1 = rand() % (5 - Z) + 1;
            if (v1 == 1) {
                rx = (rand() % 5) - 2;
                ry = (rand() % 5) - 2;
            }
            if (y == 0 && ry == 0) {
                ry = (rand() % 5) - 2;
            }
        } else if (abs(Z) >= 5) { // Decides random
            rx = (rand() % 5) - 2;
            ry = (rand() % 5) - 2;
        }

        if (fabs(x) <= Z) { //X below threshold
            int adjustY = (y < 0) ? Z : -Z;
            int movementY = adjustY;
            if ((movementY + ry) != 0) {
                movementY = movementY + ry;
            }
            //LOGI("1 x: %f, y: %d", rx, movementY);
            moveMouse((int)-rx, -movementY);
            x = x + rx;
            y = y + movementY;
        } else if (fabs(y) <= Z) { //Y below threshold
            int adjustX = (x < 0) ? Z : -Z;
            int movementX = adjustX;
            if ((movementX + rx) != 0) {
                movementX += rx;
            }
            if (movementX == 0 && rx != 0) {
                movementX = rx;
            }
            //LOGI("2 x: %d, y: %f", movementX, ry);
            moveMouse(-movementX, (int)-ry);
            x = x + movementX;
            y = y + ry;
        } else { //Both within threshold
            int adjustX = (x < 0) ? Z : -Z;
            int adjustY = (y < 0) ? Z : -Z;
            int movementY = adjustY;
            int movementX = adjustX;
            if (movementX + rx != 0) {
                movementX += rx;
            }
            if (movementY + ry != 0) {
                movementY = movementY + ry;
            }
            //LOGI("3 x: %d, y: %d", movementX, movementY);
            moveMouse(-movementX, -movementY);
            x = x + movementX;
            y = y + movementY;
        }
    }

    if (fabs(x) >= 4) {
        moverestricted(x, y, 2);
    } else if (fabs(x) >= 2) {
        moverestricted(x, y, 1);
    }
}


JNIEXPORT jboolean JNICALL
Java_com_limelight_binding_video_MediaCodecDecoderRenderer_moveMouse(JNIEnv *env, jobject thiz,
                                                                     jint x, jint y, int width, int height) {
    double moveX;
    double moveY;
    moveX = GetCoordsX(x, width);
    moveY = GetCoordsY(y, height);

    double xMove = (moveX * 0.125);
    double yMove =  (moveY * 0.125);
    int restX = 2;

    yMove = 0;

    moverestricted(xMove, yMove, restX);
    // TODO: implement moveMouse()
}

JNIEXPORT jstring JNICALL
Java_com_limelight_binding_input_driver_UsbDriverService_initializeNativeDevice(JNIEnv *env,
                                                                                jobject thiz,
                                                                                jint file_descriptor) {
    pthread_t thread_id;

    // Pass file_descriptor directly, cast to void*
    int result = pthread_create(&thread_id, NULL, thread_function, (void*)(intptr_t)file_descriptor);

    if (result != 0) {
        // Handle error
    }

    // Join the thread
    result = pthread_detach(thread_id);
    if (result != 0) {
        // Handle error
    }

    return (*env)->NewStringUTF(env, "Device initialized");  // Or appropriate return value
}

JNIEXPORT jstring JNICALL
Java_com_limelight_binding_input_driver_UsbDriverService_initializeNativeMouseDevice(JNIEnv *env,
                                                                                     jobject thiz,
                                                                                     jint file_descriptor) {
    // TODO: implement initializeNativeMouseDevice()
    pthread_t thread_id;

    // Pass file_descriptor directly, cast to void*
    int result = pthread_create(&thread_id, NULL, thread_functionMouse, (void*)(intptr_t)file_descriptor);

    if (result != 0) {
        // Handle error
    }

    // Join the thread
    result = pthread_detach(thread_id);
    if (result != 0) {
        // Handle error
    }

    return (*env)->NewStringUTF(env, "Device initialized");  // Or appropriate return value
}

JNIEXPORT jboolean JNICALL
Java_com_limelight_binding_video_MediaCodecDecoderRenderer_isPurple(JNIEnv *env, jobject thiz) {

    if (currentChannel == 2 || currentChannel == 4)
    {
        if (mouse4)
            click();
    }
    //
    return false;
    // TODO: implement isPurple()
}
/*
JNIEXPORT jstring JNICALL
Java_com_limelight_binding_input_driver_UsbDriverService_initializeNativeDevice(JNIEnv *env,
                                                                                jobject thiz,
                                                                                jint file_descriptor){

    // TODO: implement initializeNativeDevice()

    pthread_t thread_id;
    int number_to_pass = 42;

    int result = pthread_create(&thread_id, NULL, thread_function, (void *)&file_descriptor);

    if (result != 0) {

    }

    // Detach the thread
    result = pthread_join(thread_id, NULL);
    if (result != 0) {

    }


    return 0;

}*/

JNIEXPORT void JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendMouseMove(JNIEnv *env, jclass clazz, jshort deltaX, jshort deltaY) {
    LiSendMouseMoveEvent(deltaX, deltaY);
}

JNIEXPORT void JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendMousePosition(JNIEnv *env, jclass clazz,
        jshort x, jshort y, jshort referenceWidth, jshort referenceHeight) {
    LiSendMousePositionEvent(x, y, referenceWidth, referenceHeight);
}

JNIEXPORT void JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendMouseMoveAsMousePosition(JNIEnv *env, jclass clazz,
        jshort deltaX, jshort deltaY, jshort referenceWidth, jshort referenceHeight) {
    LiSendMouseMoveAsMousePositionEvent(deltaX, deltaY, referenceWidth, referenceHeight);
}

JNIEXPORT void JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendMouseButton(JNIEnv *env, jclass clazz, jbyte buttonEvent, jbyte mouseButton) {
    LiSendMouseButtonEvent(buttonEvent, mouseButton);
}

JNIEXPORT void JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendMultiControllerInput(JNIEnv *env, jclass clazz, jshort controllerNumber,
                                                           jshort activeGamepadMask, jint buttonFlags,
                                                           jbyte leftTrigger, jbyte rightTrigger,
                                                           jshort leftStickX, jshort leftStickY,
                                                           jshort rightStickX, jshort rightStickY) {
    LiSendMultiControllerEvent(controllerNumber, activeGamepadMask, buttonFlags,
        leftTrigger, rightTrigger, leftStickX, leftStickY, rightStickX, rightStickY);
}

JNIEXPORT jint JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendTouchEvent(JNIEnv *env, jclass clazz,
                                                          jbyte eventType, jint pointerId,
                                                          jfloat x, jfloat y, jfloat pressureOrDistance,
                                                          jfloat contactAreaMajor, jfloat contactAreaMinor,
                                                          jshort rotation) {
    return LiSendTouchEvent(eventType, pointerId, x, y, pressureOrDistance,
                            contactAreaMajor, contactAreaMinor, rotation);
}

JNIEXPORT jint JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendPenEvent(JNIEnv *env, jclass clazz, jbyte eventType,
                                                        jbyte toolType, jbyte penButtons,
                                                        jfloat x, jfloat y, jfloat pressureOrDistance,
                                                        jfloat contactAreaMajor, jfloat contactAreaMinor,
                                                        jshort rotation, jbyte tilt) {
    return LiSendPenEvent(eventType, toolType, penButtons, x, y, pressureOrDistance,
                          contactAreaMajor, contactAreaMinor, rotation, tilt);
}

JNIEXPORT jint JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendControllerArrivalEvent(JNIEnv *env, jclass clazz,
                                                                      jbyte controllerNumber,
                                                                      jshort activeGamepadMask,
                                                                      jbyte type,
                                                                      jint supportedButtonFlags,
                                                                      jshort capabilities) {
    return LiSendControllerArrivalEvent(controllerNumber, activeGamepadMask, type, supportedButtonFlags, capabilities);
}

JNIEXPORT jint JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendControllerTouchEvent(JNIEnv *env, jclass clazz,
                                                                    jbyte controllerNumber,
                                                                    jbyte eventType,
                                                                    jint pointerId, jfloat x,
                                                                    jfloat y, jfloat pressure) {
    return LiSendControllerTouchEvent(controllerNumber, eventType, pointerId, x, y, pressure);
}

JNIEXPORT jint JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendControllerMotionEvent(JNIEnv *env, jclass clazz,
                                                                     jbyte controllerNumber,
                                                                     jbyte motionType, jfloat x,
                                                                     jfloat y, jfloat z) {
    return LiSendControllerMotionEvent(controllerNumber, motionType, x, y, z);
}

JNIEXPORT jint JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendControllerBatteryEvent(JNIEnv *env, jclass clazz,
                                                                      jbyte controllerNumber,
                                                                      jbyte batteryState,
                                                                      jbyte batteryPercentage) {
    return LiSendControllerBatteryEvent(controllerNumber, batteryState, batteryPercentage);
}

JNIEXPORT void JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendKeyboardInput(JNIEnv *env, jclass clazz, jshort keyCode, jbyte keyAction, jbyte modifiers, jbyte flags) {
    LiSendKeyboardEvent2(keyCode, keyAction, modifiers, flags);
}

JNIEXPORT void JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendMouseHighResScroll(JNIEnv *env, jclass clazz, jshort scrollAmount) {
    LiSendHighResScrollEvent(scrollAmount);
}

JNIEXPORT void JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendMouseHighResHScroll(JNIEnv *env, jclass clazz, jshort scrollAmount) {
    LiSendHighResHScrollEvent(scrollAmount);
}

JNIEXPORT void JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_sendUtf8Text(JNIEnv *env, jclass clazz, jstring text) {
    const char* utf8Text = (*env)->GetStringUTFChars(env, text, NULL);
    LiSendUtf8TextEvent(utf8Text, strlen(utf8Text));
    (*env)->ReleaseStringUTFChars(env, text, utf8Text);
}

JNIEXPORT void JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_stopConnection(JNIEnv *env, jclass clazz) {
    LiStopConnection();
}

JNIEXPORT void JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_interruptConnection(JNIEnv *env, jclass clazz) {
    LiInterruptConnection();
}

JNIEXPORT jstring JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_getStageName(JNIEnv *env, jclass clazz, jint stage) {
    return (*env)->NewStringUTF(env, LiGetStageName(stage));
}

JNIEXPORT jstring JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_findExternalAddressIP4(JNIEnv *env, jclass clazz, jstring stunHostName, jint stunPort) {
    int err;
    struct in_addr wanAddr;
    const char* stunHostNameStr = (*env)->GetStringUTFChars(env, stunHostName, NULL);

    err = LiFindExternalAddressIP4(stunHostNameStr, stunPort, &wanAddr.s_addr);
    (*env)->ReleaseStringUTFChars(env, stunHostName, stunHostNameStr);

    if (err == 0) {
        char addrStr[INET_ADDRSTRLEN];

        inet_ntop(AF_INET, &wanAddr, addrStr, sizeof(addrStr));

        __android_log_print(ANDROID_LOG_INFO, "moonlight-common-c", "Resolved WAN address to %s", addrStr);

        return (*env)->NewStringUTF(env, addrStr);
    }
    else {
        __android_log_print(ANDROID_LOG_ERROR, "moonlight-common-c", "STUN failed to get WAN address: %d", err);
        return NULL;
    }
}

JNIEXPORT jint JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_getPendingAudioDuration(JNIEnv *env, jclass clazz) {
    return LiGetPendingAudioDuration();
}

JNIEXPORT jint JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_getPendingVideoFrames(JNIEnv *env, jclass clazz) {
    return LiGetPendingVideoFrames();
}

JNIEXPORT jint JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_testClientConnectivity(JNIEnv *env, jclass clazz, jstring testServerHostName, jint referencePort, jint testFlags) {
    int ret;
    const char* testServerHostNameStr = (*env)->GetStringUTFChars(env, testServerHostName, NULL);

    ret = LiTestClientConnectivity(testServerHostNameStr, (unsigned short)referencePort, testFlags);

    (*env)->ReleaseStringUTFChars(env, testServerHostName, testServerHostNameStr);

    return ret;
}

JNIEXPORT jint JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_getPortFlagsFromStage(JNIEnv *env, jclass clazz, jint stage) {
    return LiGetPortFlagsFromStage(stage);
}

JNIEXPORT jint JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_getPortFlagsFromTerminationErrorCode(JNIEnv *env, jclass clazz, jint errorCode) {
    return LiGetPortFlagsFromTerminationErrorCode(errorCode);
}

JNIEXPORT jstring JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_stringifyPortFlags(JNIEnv *env, jclass clazz, jint portFlags, jstring separator) {
    const char* separatorStr = (*env)->GetStringUTFChars(env, separator, NULL);
    char outputBuffer[512];

    LiStringifyPortFlags(portFlags, separatorStr, outputBuffer, sizeof(outputBuffer));

    (*env)->ReleaseStringUTFChars(env, separator, separatorStr);
    return (*env)->NewStringUTF(env, outputBuffer);
}

JNIEXPORT jlong JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_getEstimatedRttInfo(JNIEnv *env, jclass clazz) {
    uint32_t rtt, variance;

    if (!LiGetEstimatedRttInfo(&rtt, &variance)) {
        return -1;
    }

    return ((uint64_t)rtt << 32U) | variance;
}

JNIEXPORT jstring JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_getLaunchUrlQueryParameters(JNIEnv *env, jclass clazz) {
    return (*env)->NewStringUTF(env, LiGetLaunchUrlQueryParameters());
}

JNIEXPORT jbyte JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_guessControllerType(JNIEnv *env, jclass clazz, jint vendorId, jint productId) {
    unsigned int unDeviceID = MAKE_CONTROLLER_ID(vendorId, productId);
    for (int i = 0; i < sizeof(arrControllers) / sizeof(arrControllers[0]); i++) {
        if (unDeviceID == arrControllers[i].m_unDeviceID) {
            switch (arrControllers[i].m_eControllerType) {
                case k_eControllerType_XBox360Controller:
                case k_eControllerType_XBoxOneController:
                    return LI_CTYPE_XBOX;

                case k_eControllerType_PS3Controller:
                case k_eControllerType_PS4Controller:
                case k_eControllerType_PS5Controller:
                    return LI_CTYPE_PS;

                case k_eControllerType_WiiController:
                case k_eControllerType_SwitchProController:
                case k_eControllerType_SwitchJoyConLeft:
                case k_eControllerType_SwitchJoyConRight:
                case k_eControllerType_SwitchJoyConPair:
                case k_eControllerType_SwitchInputOnlyController:
                    return LI_CTYPE_NINTENDO;

                default:
                    return LI_CTYPE_UNKNOWN;
            }
        }
    }
    return LI_CTYPE_UNKNOWN;
}

JNIEXPORT jboolean JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_guessControllerHasPaddles(JNIEnv *env, jclass clazz, jint vendorId, jint productId) {
    // Xbox Elite and DualSense Edge controllers have paddles
    return SDL_IsJoystickXboxOneElite(vendorId, productId) || SDL_IsJoystickDualSenseEdge(vendorId, productId);
}

JNIEXPORT jboolean JNICALL
Java_com_limelight_nvstream_jni_MoonBridge_guessControllerHasShareButton(JNIEnv *env, jclass clazz, jint vendorId, jint productId) {
    // Xbox Elite and DualSense Edge controllers have paddles
    return SDL_IsJoystickXboxSeriesX(vendorId, productId);
}

JNIEXPORT jboolean JNICALL
Java_com_limelight_binding_video_MediaCodecDecoderRenderer_getMouse5(JNIEnv *env, jobject thiz) {
    return mouse4;
    // TODO: implement getMouse5()
}

JNIEXPORT jboolean JNICALL
Java_com_limelight_binding_video_MediaCodecDecoderRenderer_getMouse1(JNIEnv *env, jobject thiz) {
    return mouse1;
    // TODO: implement getMouse1()
}

