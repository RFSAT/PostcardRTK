// End-to-end check of the NTRIP request/response code and RTCM framer against tools/fake_caster.py.
// Usage: caster_e2e <port> <mount> [user:pass]   -> prints the reply class and frames received in 3 s
#include "ntrip_proto.h"
#include "rtcm.h"
#include <arpa/inet.h>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <unistd.h>
int main(int argc, char **argv)
{
    int port = atoi(argv[1]);
    std::string mount = argv[2], user, pass;
    if (argc > 3) { std::string up = argv[3]; auto c = up.find(':'); user = up.substr(0, c); pass = up.substr(c + 1); }
    int s = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(port); inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
    if (connect(s, (sockaddr *)&a, sizeof(a))) { puts("connect failed"); return 2; }
    char req[600];
    size_t n = ntripBuildRequest(req, sizeof(req), "127.0.0.1", port, mount.c_str(), user.c_str(), pass.c_str(), false, "PostcardRTK/test");
    write(s, req, n);
    timeval tv{0, 200000}; setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    NtripResponse r; r.reset(); RtcmFramer f; NtripReply res = NtripReply::Incomplete;
    time_t end = time(nullptr) + 3; uint8_t buf[2048];
    while (time(nullptr) < end) {
        ssize_t k = read(s, buf, sizeof(buf));
        if (k == 0) break;
        if (k < 0) continue;
        size_t off = 0;
        if (res == NtripReply::Incomplete) { res = r.feed(buf, k, &off); if (res != NtripReply::Ok) { if (res != NtripReply::Incomplete) break; continue; } }
        for (size_t i = off; i < (size_t)k; i++) f.feed(buf[i]);
    }
    const char *names[] = {"Incomplete", "Ok", "Sourcetable", "Unauthorized", "SoftError", "HardError"};
    printf("%s frames=%u crcErrors=%u station=%d\n", names[(int)res], f.frames(), f.crcErrors(), f.stationId());
    return 0;
}
