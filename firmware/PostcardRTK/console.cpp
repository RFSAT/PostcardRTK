// console.cpp - command line on the Postcard's USB port A (115200 baud), for setting it up without Wi-Fi.
// Type "help". Settings use the same JSON as the web page's settings file.
#include "console.h"
#include "config.h"
#include "eventlog.h"
#include "net.h"
#include "ntrip.h"
#include "receiver.h"
#include "shared.h"
#include "util.h"
#include <Arduino.h>

static String line;

static void help()
{
    Serial.print(
        "PostcardRTK commands:\r\n"
        "  status                  one-line status\r\n"
        "  ladder                  what the method ladder decided and why\r\n"
        "  config                  settings as JSON (passwords hidden)\r\n"
        "  set {json}              merge settings, e.g. set {\"wifi\":[{\"ssid\":\"Home\",\"pass\":\"secret\"}]}\r\n"
        "  wifi <ssid> <password>  quick: make this the first Wi-Fi network\r\n"
        "  save                    store the settings (restart to apply)\r\n"
        "  rx <command>            send a command to the LG290P, e.g. rx $PQTMVERNO (checksum added)\r\n"
        "  replies                 the receiver's latest command replies\r\n"
        "  events                  recent events\r\n"
        "  ap                      open the setup access point for 10 minutes\r\n"
        "  restart                 restart the Postcard\r\n");
}

static void doSet(const String &json)
{
    String err;
    if (configFromJson(json, err))
        Serial.print("ok, type save then restart to apply\r\n");
    else
        Serial.printf("not changed: %s\r\n", err.c_str());
}

static void doWifi(const String &args)
{
    int sp = args.indexOf(' ');
    String ssid = sp < 0 ? args : args.substring(0, sp);
    String pass = sp < 0 ? String("") : args.substring(sp + 1);
    if (!ssid.length())
    {
        Serial.print("usage: wifi <ssid> <password>\r\n");
        return;
    }
    for (int i = CFG_MAX_WIFI - 1; i > 0; i--)
        cfg.wifi[i] = cfg.wifi[i - 1];
    cfg.wifi[0].ssid = ssid;
    cfg.wifi[0].pass = pass;
    for (int i = 1; i < CFG_MAX_WIFI; i++)
        if (cfg.wifi[i].ssid == ssid)
            cfg.wifi[i] = WifiNet();
    Serial.printf("Wi-Fi 1 = %s; type save then restart to apply\r\n", ssid.c_str());
}

static void doRx(const String &cmd)
{
    char out[140];
    size_t n = nmeaFinish(cmd.c_str(), out, sizeof(out));
    if (!n)
    {
        Serial.print("usage: rx $PQTM...\r\n");
        return;
    }
    cmdToRx.write((const uint8_t *)out, n);
    Serial.printf("sent %s", out);
}

static void doReplies()
{
    mutex_enter_blocking(&typesMutex);
    uint32_t seq = rxReplySeq;
    int count = seq < (uint32_t)RX_REPLIES ? (int)seq : RX_REPLIES;
    for (int i = count; i > 0; i--)
        Serial.printf("%s\r\n", rxReplies[(seq - i) % RX_REPLIES]);
    mutex_exit(&typesMutex);
    if (!count)
        Serial.print("no replies yet\r\n");
}

static void doLadder()
{
    static Ladder l; // copied under the receiver's lock
    getLadder(l);
    Serial.printf("ladder: %s - %s\r\n", ladderStateName(l.state()), l.reason());
    if (l.state() == LS_RTK_PAUSED)
        Serial.printf("  RTK retried in %lu s\r\n", (unsigned long)((l.pausedUntilMs() - millis()) / 1000));
    if (l.altBest() >= 0)
        Serial.printf("  best Galileo HAS / SBAS estimate here in the last hour: %.2f m\r\n", l.altBest());
}

static void run(String cmd)
{
    cmd.trim();
    if (!cmd.length())
        return;
    int sp = cmd.indexOf(' ');
    String word = sp < 0 ? cmd : cmd.substring(0, sp);
    String rest = sp < 0 ? String("") : cmd.substring(sp + 1);
    rest.trim();
    word.toLowerCase();
    if (word == "help" || word == "?")
        help();
    else if (word == "status")
        consolePrintStatus();
    else if (word == "ladder")
        doLadder();
    else if (word == "config")
        Serial.print(configToJson(false) + "\r\n");
    else if (word == "set")
        doSet(rest);
    else if (word == "wifi")
        doWifi(rest);
    else if (word == "save")
        Serial.print(configSave() ? "saved\r\n" : "could not save\r\n");
    else if (word == "rx")
        doRx(rest);
    else if (word == "replies")
        doReplies();
    else if (word == "events")
    {
        String t;
        eventsText(t);
        if (t.length() > 4000)
            t = t.substring(t.length() - 4000);
        t.replace("\t", "  ");
        t.replace("\n", "\r\n");
        Serial.print(t + "\r\n");
    }
    else if (word == "ap")
        netStartAp();
    else if (word == "restart" || word == "reboot")
        safeReboot();
    else
        Serial.printf("unknown command '%s', type help\r\n", word.c_str());
}

void consoleLoop()
{
    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n')
        {
            if (line.length())
            {
                Serial.print("> " + line + "\r\n");
                run(line);
            }
            line = "";
        }
        else if (c == 8 || c == 127)
        {
            if (line.length())
                line.remove(line.length() - 1);
        }
        else if (line.length() < 1500)
            line += c;
    }
}
