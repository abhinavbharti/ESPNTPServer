/*
 * ESPNTPServer.cpp
 *
 * Copyright 2017 Christopher B. Liebman
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 *  Created on: Oct 29, 2017
 *      Author: liebman
 */

#include "ESPNTPServer.h"
#define DEBUG
//define NTP_PACKET_DEBUG
#include "Logger.h"
#include <lwip/def.h> // htonl() & ntohl()
#include "freertos/FreeRTOS.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/igmp.h"
#include "lwip/udp.h"
#include "lwip/tcpip.h"
#include "lwip/priv/tcpip_priv.h"

Ticker            validityTimer;
volatile bool     valid;
volatile bool     gps_valid;
uint32_t          valid_in;
volatile time_t   since;
volatile char     reason[128];
time_t            first_gps_valid;
volatile uint32_t valid_count;  // how many times we have gone from invalid to valid
bool              sentence_unknown;
uint32_t          bad_checksum_count;
volatile uint32_t req_count;
volatile uint32_t rsp_count;
int8_t            precision;
volatile uint32_t dispersion;

volatile time_t   seconds;

volatile uint32_t last_micros;
volatile uint32_t min_micros;
volatile uint32_t max_micros;

#if defined(ESP8266)
#if defined(USE_ASYNC_UDP)
AsyncUDP udp;
#else
WiFiUDP udp;
#endif
#endif

#if defined(ESP_PLATFORM)
HardwareSerial gps(2);
#else
SoftwareSerial gps(GPS_RX_PIN, GPS_TX_PIN, false, SERIAL_BUFFER_SIZE);
#endif
char           nmeaBuffer[NMEA_BUFFER_SIZE];
MicroNMEA      nmea(nmeaBuffer, NMEA_BUFFER_SIZE);

SSD1306Wire    display(0x3c, SDA, SCL);
volatile bool  display_now;

#ifdef NTP_PACKET_DEBUG
void dumpNTPPacket(NTPPacket* ntp)
{
    dbprintf("size:       %u\n", sizeof(*ntp));
    dbprintf("firstbyte:  0x%02x\n", *(uint8_t*)ntp);
    dbprintf("li:         %u\n", getLI(ntp->flags));
    dbprintf("version:    %u\n", getVERS(ntp->flags));
    dbprintf("mode:       %u\n", getMODE(ntp->flags));
    dbprintf("stratum:    %u\n", ntp->stratum);
    dbprintf("poll:       %u\n", ntp->poll);
    dbprintf("precision:  %d\n", ntp->precision);
    dbprintf("delay:      %u\n", ntp->delay);
    dbprintf("dispersion: %u\n", ntp->dispersion);
    dbprintf("ref_id:     %02x:%02x:%02x:%02x\n", ntp->ref_id[0], ntp->ref_id[1], ntp->ref_id[2], ntp->ref_id[3]);
    dbprintf("ref_time:   %08x:%08x\n", ntp->ref_time.seconds, ntp->ref_time.fraction);
    dbprintf("orig_time:  %08x:%08x\n", ntp->orig_time.seconds, ntp->orig_time.fraction);
    dbprintf("recv_time:  %08x:%08x\n", ntp->recv_time.seconds, ntp->recv_time.fraction);
    dbprintf("xmit_time:  %08x:%08x\n", ntp->xmit_time.seconds, ntp->xmit_time.fraction);
}
#else
#define dumpNTPPacket(x)
#endif

void invalidateTime();

void IRAM_ATTR invalidate(const char* fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    if (reason[0] == '\0')
    {
        vsnprintf((char*)reason, sizeof(reason)-1, fmt, argp);
    }
    va_end(argp);

    if (valid)
    {
        since = seconds;
    }

    valid       = false;
    gps_valid   = false;
    last_micros = 0;
    valid_in    = 0;
    nmea.clear();

    display_now = true;
}

void IRAM_ATTR invalidateTime()
{
    invalidate("Timer: %f\n", us2s(micros()-last_micros));
    //
    // restart the validity timer, if it runs out we invalidate our data.
    //
    validityTimer.attach_ms(VALIDITY_CHECK_MS, &invalidateTime);
}


void IRAM_ATTR oneSecondInterrupt()
{
#if 1
    digitalWrite(LED_PIN, HIGH);
#endif
    uint32_t cur_micros = micros();

    //
    // restart the validity timer, if it runs out we invalidate our data.
    //
    validityTimer.attach_ms(VALIDITY_CHECK_MS, &invalidateTime);

    //
    // don't trust PPS if GPS is not valid.
    //
    if (!gps_valid)
    {
        return;
    }

    display_now = true;

    //
    // if we are still counting down then keep waiting
    //
    if (valid_in)
    {
        --valid_in;
        if (valid_in == 0)
        {
            // clear stats and mark us valid
            min_micros  = 0;
            max_micros  = 0;
            valid       = true;
            since       = seconds;
            ++valid_count;
            reason[0] = '\0';
        }
    }

    //
    // increment seconds
    //
    seconds += 1;

    //
    // the first time around we just initialize the last value
    //
    if (last_micros == 0)
    {
        last_micros = cur_micros;
        return;
    }

    uint32_t micros_count = cur_micros - last_micros;
    last_micros           = cur_micros;

    if (min_micros == 0 || micros_count < min_micros)
    {
        min_micros = micros_count;
    }

    if (micros_count > max_micros)
    {
        max_micros = micros_count;
    }

#if 1
    digitalWrite(LED_PIN, LOW);
#else
#if defined(DEBUG) && defined(LED_PIN)
    digitalWrite(LED_PIN, digitalRead(LED_PIN) ? LOW : HIGH);
#endif
#endif
}

void IRAM_ATTR getNTPTime(NTPTime *time)
{
    time->seconds = toNTP(seconds);
    uint32_t micros_delta = micros() - last_micros;

    //
    // if micros_delta is at or bigger than one second then
    // use the max fraction.
    //
    if (micros_delta >= 1000000)
    {
        time->fraction = 0xffffffff;
        return;
    }

    time->fraction = (uint32_t)(us2s(micros_delta) * (double)4294967296L);
}

int8_t computePrecision()
{
    NTPTime t;
    unsigned long start = micros();
    for (int i = 0; i < PRECISION_COUNT; ++i)
    {
        getNTPTime(&t);
    }
    unsigned long end   = micros();
    double        total = (double)(end - start) / 1000000.0;
    double        time  = total / PRECISION_COUNT;
    double        prec  = log2(time);
    dbprintf("computePrecision: total:%f time:%f prec:%f\n", total, time, prec);
    return (int8_t)prec;
}

#if defined(ESP8266)
#if defined(USE_ASYNC_UDP)
void IRAM_ATTR recievePacket(AsyncUDPPacket aup)
#else
void recievePacket()
#endif
{
    ++req_count;
    NTPPacket ntp;
    NTPTime   recv_time;
    getNTPTime(&recv_time);
#if defined(USE_ASYNC_UDP)
    if (aup.length() != sizeof(NTPPacket))
    {
        dbprintf("recievePacket: ignoring packet with bad length: %d < %d\n", aup.length(), sizeof(NTPPacket));
        return;
    }
#else
    if (udp.available() != sizeof(NTPPacket))
    {
        dbprintf("recievePacket: ignoring packet with bad length: %d < %d\n", udp.available(), sizeof(NTPPacket));
        udp.flush();
        return;
    }
#endif

    if (!valid)
    {
        dbprintln("recievePacket: GPS data not valid!");
#if !defined(USE_ASYNC_UDP)
        udp.flush();
#endif
        return;
    }

#if defined(USE_ASYNC_UDP)
    memcpy(&ntp, aup.data(), sizeof(ntp));
#else
    udp.read((unsigned char*)&ntp, sizeof(ntp));
#endif

    ntp.delay              = ntohl(ntp.delay);
    ntp.dispersion         = ntohl(ntp.dispersion);
    ntp.orig_time.seconds  = ntohl(ntp.orig_time.seconds);
    ntp.orig_time.fraction = ntohl(ntp.orig_time.fraction);
    ntp.ref_time.seconds   = ntohl(ntp.ref_time.seconds);
    ntp.ref_time.fraction  = ntohl(ntp.ref_time.fraction);
    ntp.recv_time.seconds  = ntohl(ntp.recv_time.seconds);
    ntp.recv_time.fraction = ntohl(ntp.recv_time.fraction);
    ntp.xmit_time.seconds  = ntohl(ntp.xmit_time.seconds);
    ntp.xmit_time.fraction = ntohl(ntp.xmit_time.fraction);
    dumpNTPPacket(&ntp);

    //
    // Build the response
    //
    ntp.flags      = setLI(LI_NONE) | setVERS(NTP_VERSION) | setMODE(MODE_SERVER);
    ntp.stratum    = 1;
    ntp.precision  = precision;
    // TODO: compute actual root delay, and root dispersion
    ntp.delay = (uint32_t)(0.000001 * 65536);
    ntp.dispersion = dispersion;
    strncpy((char*)ntp.ref_id, REF_ID, sizeof(ntp.ref_id));
    ntp.orig_time  = ntp.xmit_time;
    ntp.recv_time  = recv_time;
    getNTPTime(&(ntp.ref_time));
    dumpNTPPacket(&ntp);
    ntp.delay              = htonl(ntp.delay);
    ntp.dispersion         = htonl(ntp.dispersion);
    ntp.orig_time.seconds  = htonl(ntp.orig_time.seconds);
    ntp.orig_time.fraction = htonl(ntp.orig_time.fraction);
    ntp.ref_time.seconds   = htonl(ntp.ref_time.seconds);
    ntp.ref_time.fraction  = htonl(ntp.ref_time.fraction);
    ntp.recv_time.seconds  = htonl(ntp.recv_time.seconds);
    ntp.recv_time.fraction = htonl(ntp.recv_time.fraction);
    getNTPTime(&(ntp.xmit_time));
    ntp.xmit_time.seconds  = htonl(ntp.xmit_time.seconds);
    ntp.xmit_time.fraction = htonl(ntp.xmit_time.fraction);
#if defined(USE_ASYNC_UDP)
    aup.write((uint8_t*)&ntp, sizeof(ntp));
#else
    IPAddress address = udp.remoteIP();
    uint16_t port     = udp.remotePort();
    udp.beginPacket(address, port);
    udp.write((uint8_t*)&ntp, sizeof(ntp));
    udp.endPacket();
#endif
    ++rsp_count;
}
#endif

void badChecksum(MicroNMEA& mn)
{
    ++bad_checksum_count;
    dbprintf("badChecksum: '%s'\n", mn.getSentence());
}

void unknownSentence(MicroNMEA& mn)
{
    const char* sentence = mn.getSentence();

    //
    // 103 - about to restart
    // 105 - restart complete
    //
    if (!strncmp(sentence, "$PGACK,103*40", 13) || !strncmp(sentence, "$PGACK,105*46", 13))
    {
        invalidate("restarting: %s", sentence);
        return;
    }

    //
    // ignore startup seq.
    //
    if (!strncmp(sentence, "$PMTK011,MTKGPS*08", 18) || !strncmp(sentence, "$PMTK010,001*2E", 15))
    {
        return;
    }

    //
    // ack for '$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
    //
    if (!strncmp(sentence, "$PMTK001,314,3*36", 16))
    {
        dbprintf("sentence acked: %s\n", sentence);
        return;
    }

    if (!strncmp(sentence, "$PMTK010,002*2D", 15))
    {
        dbprintf("gps startup complete: %s\n", sentence);
        sentence_unknown = true;
        return;
    }

    if (!strncmp("$PGACK", sentence, 6))
    {
        dbprintf("ack recieved: %s\n", sentence);
        return;
    }

    dbprintf("unknownSentence: %s\n", sentence);

    sentence_unknown = true;
}

void sendSentence(const char* sentence)
{
    static char cksum[3];
    MicroNMEA::generateChecksum(sentence, cksum);
    cksum[2] = '\0';
    dbprintf("sendSentence: '%s*%s'\n", sentence, cksum);
    gps.printf("%s*%s\r\n", sentence, cksum);
#if !defined(AVOID_FLUSH)
    gps.flush();
#endif
}

void resetGPS()
{
    dbprintln("resetGPS: starting!");
    // Empty input buffer
    while (gps.available())
    {
        gps.read();
    }

    digitalWrite(GPS_EN_PIN, LOW);
    delay(100);
    digitalWrite(GPS_EN_PIN, HIGH);

    dbprintln("resetGPS: waiting on first sentence");
    dbflush();

    // Reset is complete when the first valid message is received
    while (1)
    {
        delay(1);
        while (gps.available())
        {
            char c = gps.read();
            if (nmea.process(c))
            {
                const char* sentence = nmea.getSentence();
                dbprintf("resetGPS: done, sentence: '%s'\n", sentence);
#if 0
                dbprintln("setting baud rate to 38400!");
                sendSentence("$PMTK251,38400");
#if !defined(AVOID_FLUSH)
                gps.flush();
#endif
                gps.begin(38400);
#endif
                return;
            }
        }
    }
}

void processGPS()
{
    static boolean last_valid     = false;
    static boolean last_gps_valid = false;

    //
    // Print valid or invalid if stats has changed.
    //
    if ((last_valid && !valid) || (last_gps_valid && !gps_valid))
    {
        dbprintf("INVALID: '%s'\n", reason);
        reason[0] = '\0';
    }
    else if (!last_valid && valid)
    {
        dbprintln("VALID!");
    }
    last_valid = valid;
    last_gps_valid = gps_valid;

    while (gps.available() > 0)
    {
        if (nmea.process(gps.read()))
        {
            //dbprintf("sentence: %s\n", nmea.getSentence());
            //
            // if it was a GGA and its valid then check and maybe update the time
            //
            const char * id = nmea.getMessageID();
            if (nmea.isValid() && nmea.getYear() > 2000 && strcmp("GGA", id) == 0)
            {
                static bool timewarp = false;  // true if prev pass was timewarp
                static struct tm tm;
                tm.tm_year         = nmea.getYear() - 1900;
                tm.tm_mon          = nmea.getMonth() - 1;
                tm.tm_mday         = nmea.getDay();
                tm.tm_hour         = nmea.getHour();
                tm.tm_min          = nmea.getMinute();
                tm.tm_sec          = nmea.getSecond();

                time_t new_seconds = mktime(&tm);
                time_t old_seconds = seconds;

                if (old_seconds < new_seconds)
                {
                    if (valid)
                    {
                        invalidate("%ld missing seconds!", new_seconds-old_seconds);
                    }
                    seconds = new_seconds;
                    dbprintf("%010lu: %s adjusting seconds from %lu to %lu\n", millis(), nmea.getMessageID(), old_seconds, new_seconds);
                }
                else if (old_seconds > new_seconds)
                {
                    if (valid)
                    {
                        if (timewarp)
                        {
                            invalidate("timewarp %ld", new_seconds-old_seconds);
                            timewarp = false;
                        }
                        else
                        {
                            timewarp = true;
                        }
                    }
                    else
                    {
                        seconds = new_seconds;
                    }
                }

                //
                // if gps was not valid, it is now
                //
                if (!gps_valid)
                {
                    gps_valid       = true;
                    valid_in = PPS_VALID_COUNT;
                    if (!first_gps_valid)
                    {
                        first_gps_valid = seconds;
                    }
                    dbprintln("gps valid!");
                }
            }
        }
    }
}

void updateDisplay()
{
    static bool blink;
    const int buf_size = 80;
    char buf[buf_size];
    struct tm t;

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    time_t now = seconds;
    gmtime_r(&now, &t);
    snprintf(buf, buf_size-1, "UTC: %04d/%02d/%02d %02d:%02d:%02d", t.tm_year+1900, t.tm_mon+1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
    display.drawString(0, 0,  buf);
    display.drawString(0, 10, "Address: "+WiFi.localIP().toString());
    display.drawString(0, 20, "R&R:  " + String(req_count) + " / " + String(rsp_count));
    display.drawString(0, 30, "Sat Count: " + String(nmea.getNumSatellites()));

    if (valid_in)
    {
        snprintf(buf, buf_size-1, "valid in: %02u", valid_in);
        String value(buf);
        uint16_t w = display.getStringWidth(value);
        display.drawString(128-w, 30, value);
    }

    gmtime_r(&first_gps_valid, &t);
    snprintf(buf, buf_size-1, "%04d/%02d/%02d %02d:%02d:%02d", t.tm_year+1900, t.tm_mon+1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
    display.drawString(0, 40, "Start: " + String(buf));
    gmtime_r((time_t*)&since, &t);
    if (valid)
    {
        snprintf(buf, buf_size-1, "Valid: %04d/%02d/%02d %02d:%02d:%02d", t.tm_year+1900, t.tm_mon+1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
        display.drawString(0, 50, String(buf));
    }
    else
    {
        if (blink)
        {
            snprintf(buf, buf_size-1, "Invld: %04d/%02d/%02d %02d:%02d:%02d", t.tm_year+1900, t.tm_mon+1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
            display.drawString(0, 50, String(buf));
        }
        blink = !blink;
    }
    // write the buffer to the display
    display.display();
}

String constructSSID()
{
#if defined(ESP_PLATFORM)
    uint8_t baseMac[6];
    // Get MAC address for WiFi station
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    char baseMacChr[18] = {0};
    sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    String ssid = String("ESPNTPServer") + baseMacChr;
#else
    String ssid = "ESPNTPServer" + String(ESP.getChipId());
#endif
    return ssid;
}


void startWiFi()
{
#if !defined(USE_NO_WIFI)
    WiFi.begin();
    bool auto_reconnect = WiFi.getAutoReconnect();
    if (!auto_reconnect)
    {
        dbprintln("setting auto-reconnect true!\n");
        WiFi.setAutoReconnect(true);
    }
    WiFiManager wifi;
    //wifi.setDebugOutput(false);
    String ssid = constructSSID();
    wifi.autoConnect(ssid.c_str(), NULL);
#if !defined(ESP_PLATFORM)
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
#endif
#if defined(LOG_HOST) && defined(LOG_PORT)
    dbnetlog(LOG_HOST, LOG_PORT);
    dbprintln("Network logging started!");
#endif
#endif
}

#if defined(ESP_PLATFORM)


void ntpTask(void *pvParameters)
{
    static NTPPacket ntp;
    static int udp_server = -1;

    dbprintln("ntpTask: starting!");

    if ((udp_server=socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        dbprintf("ntpTask: could not create socket: %d", errno);
        return;
    }

    int yes = 1;
    if (setsockopt(udp_server,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes)) < 0)
    {
        dbprintf("ntpTask: could not set socket option: %d", errno);
        return;
    }

    struct sockaddr_in addr;
    memset((char *) &addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(NTP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if(bind(udp_server , (struct sockaddr*)&addr, sizeof(addr)) == -1)
    {
        dbprintf("could not bind socket: %d", errno);
        return;
    }
    for(;;)
    {
        int len;
        int slen = sizeof(addr);
        if ((len = recvfrom(udp_server, &ntp, sizeof(ntp), 0, (struct sockaddr *) &addr, (socklen_t *)&slen)) == -1)
        {
            dbprintf("ntpTask: could not receive data: %d", errno);
            continue;
        }

        ++req_count;

        NTPTime   recv_time;
        getNTPTime(&recv_time);

        if (len != sizeof(NTPPacket))
        {
            dbprintf("ntpTask: ignoring packet with bad length: %d < %d\n", len, sizeof(ntp));
            continue;
        }

        if (!valid)
        {
            dbprintln("recievePacket: GPS data not valid!");
            display_now = true;
            continue;
        }

        ntp.delay              = ntohl(ntp.delay);
        ntp.dispersion         = ntohl(ntp.dispersion);
        ntp.orig_time.seconds  = ntohl(ntp.orig_time.seconds);
        ntp.orig_time.fraction = ntohl(ntp.orig_time.fraction);
        ntp.ref_time.seconds   = ntohl(ntp.ref_time.seconds);
        ntp.ref_time.fraction  = ntohl(ntp.ref_time.fraction);
        ntp.recv_time.seconds  = ntohl(ntp.recv_time.seconds);
        ntp.recv_time.fraction = ntohl(ntp.recv_time.fraction);
        ntp.xmit_time.seconds  = ntohl(ntp.xmit_time.seconds);
        ntp.xmit_time.fraction = ntohl(ntp.xmit_time.fraction);
        dumpNTPPacket(&ntp);

        //
        // Build the response
        //
        ntp.flags      = setLI(LI_NONE) | setVERS(NTP_VERSION) | setMODE(MODE_SERVER);
        ntp.stratum    = 1;
        ntp.precision  = precision;
        // TODO: compute actual root delay, and root dispersion
        ntp.delay = (uint32_t)(0.000001 * 65536);
        ntp.dispersion = dispersion;
        strncpy((char*)ntp.ref_id, REF_ID, sizeof(ntp.ref_id));
        ntp.orig_time  = ntp.xmit_time;
        ntp.recv_time  = recv_time;
        getNTPTime(&(ntp.ref_time));
        dumpNTPPacket(&ntp);
        ntp.delay              = htonl(ntp.delay);
        ntp.dispersion         = htonl(ntp.dispersion);
        ntp.orig_time.seconds  = htonl(ntp.orig_time.seconds);
        ntp.orig_time.fraction = htonl(ntp.orig_time.fraction);
        ntp.ref_time.seconds   = htonl(ntp.ref_time.seconds);
        ntp.ref_time.fraction  = htonl(ntp.ref_time.fraction);
        ntp.recv_time.seconds  = htonl(ntp.recv_time.seconds);
        ntp.recv_time.fraction = htonl(ntp.recv_time.fraction);
        getNTPTime(&(ntp.xmit_time));
        ntp.xmit_time.seconds  = htonl(ntp.xmit_time.seconds);
        ntp.xmit_time.fraction = htonl(ntp.xmit_time.fraction);

        int sent = sendto(udp_server, &ntp, sizeof(ntp), 0, (struct sockaddr*) &addr, sizeof(addr));
        if(sent < 0)
        {
          dbprintf("could not send data: %d", errno);
          continue;
        }
        ++rsp_count;
    }
}

void displayTask(void *pvParameters)
{
    dbprintln("displayTask: starting!");
    for(;;)
    {
        if (display_now)
        {
            display_now = false;
            updateDisplay();
        }
        delay(10);
    }
}
#endif

void setup()
{
    delay(5000); // delay for IDE to re-open serial
    gps.begin(9600);
    dbbegin(115200);
    dbprintln("\n\nStartup!\n");

    pinMode(PPS_PIN, INPUT);
#if defined(LED_PIN)
    pinMode(LED_PIN, OUTPUT);
#endif

    valid       = false;
    valid_count = 0;
    seconds     = 0;
    max_micros  = 0;
    min_micros  = 0;
    last_micros = 0;

    startWiFi();

    if (!display.init())
    {
        dbprintln("display.init() failed!");
    }
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.clear();
    display.drawString(0, 0, "Starting GPS...");
    display.display();

    nmea.setBadChecksumHandler(&badChecksum);
    nmea.setUnknownSentenceHandler(&unknownSentence);
    resetGPS();

    display.drawString(0, 10, "Done!");
    display.display();

    precision = computePrecision();
#if !defined(USE_NO_WIFI)
    //
    // initialize UDP handler
    //
#if defined(ESP8266)
#if defined(USE_ASYNC_UDP)
    while (!udp.listen(NTP_PORT))
    {
#else
        while(!udp.begin(NTP_PORT))
        {
#endif
        dbprintf("setup: failed to listen on port %d!  Will retry in a bit...\n", NTP_PORT);
        delay(1000);
        dbprintf("setup: retrying!\n");
    }
#endif
#endif

    attachInterrupt(PPS_PIN, &oneSecondInterrupt, FALLING);

#if defined(USE_ASYNC_UDP)
    udp.onPacket(recievePacket);
#endif

    // start teh validity timer, it also schedules a display
    validityTimer.attach_ms(VALIDITY_CHECK_MS, &invalidateTime);
    display_now = true;
    first_gps_valid = 0;
    reason[0] = '\0';
    strncpy((char*)reason, "<unknown>", sizeof(reason)-1);

#if defined(ESP_PLATFORM)
    xTaskCreatePinnedToCore(ntpTask,     "ntpTask",     8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(displayTask, "displayTask", 8192, NULL, 1, NULL, 1);
#endif
}

void loop()
{
#if !defined(USE_NO_WIFI)
    if (!WiFi.isConnected())
    {
        dbprintln("WiFi connection lost, restarting!");
        startWiFi();
    }
#endif

#if !defined(USE_ASYNC_UDP)  && defined(ESP8266)
    if (udp.parsePacket())
    {
        recievePacket();
    }
#endif

    processGPS();
    if (sentence_unknown)
    {
        sentence_unknown = false;
        // Send only RMC and GGA messages.
        sendSentence("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
    }

    static time_t last_seconds;
    if (seconds != last_seconds)
    {
        if (seconds != last_seconds && ((seconds % 60) == 0 || valid_in))
        {
            double disp = us2s(MAX(abs(MICROS_PER_SEC-max_micros), abs(MICROS_PER_SEC-min_micros)));
            dispersion  = (uint32_t)(disp * 65536.0);
            dbprintf("min:%lu max:%lu jitter:%lu valid_count:%lu valid:%s numsat:%d heap:%ld valid_in:%d badcs: %lu\n", min_micros, max_micros,
                    max_micros - min_micros, valid_count, valid ? "true" : "false", nmea.getNumSatellites(), ESP.getFreeHeap(), valid_in,
                    bad_checksum_count);
        }

        if (seconds < last_seconds)
        {
            dbprintf("OOPS: time went backwards: last:%lu now:%lu\n", last_seconds, seconds);
        }

    }

    last_seconds = seconds;

#if !defined(ESP_PLATFORM)
    if (display_now)
    {
        display_now = false;
        updateDisplay();
    }
#endif
    delay(1);
}
