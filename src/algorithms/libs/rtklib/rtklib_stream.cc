/*!
 * \file rtklib_stream.cc
 * \brief streaming functions
 * \authors <ul>
 *          <li> 2007-2013, T. Takasu
 *          <li> 2017, Javier Arribas
 *          <li> 2017, Carles Fernandez
 *          </ul>
 *
 * This is a derived work from RTKLIB http://www.rtklib.com/
 * The original source code at https://github.com/tomojitakasu/RTKLIB is
 * released under the BSD 2-clause license with an additional exclusive clause
 * that does not apply here. This additional clause is reproduced below:
 *
 * " The software package includes some companion executive binaries or shared
 * libraries necessary to execute APs on Windows. These licenses succeed to the
 * original ones of these software. "
 *
 * Neither the executive binaries nor the shared libraries are required by, used
 * or included in GNSS-SDR.
 *
 * -----------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * -----------------------------------------------------------------------------
 */

#include "rtklib_stream.h"
#include "gnss_sdr_make_unique.h"      // for std::make:unique in C++11
#include "gnss_sdr_string_literals.h"  // for std::string_literals
#include "rtklib_rtkcmn.h"
#include "rtklib_solution.h"
#include <arpa/inet.h>
#include <cctype>
#include <cerrno>
#include <cinttypes>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <memory>
#include <netdb.h>
#include <netinet/tcp.h>
#include <regex>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

using namespace std::string_literals;

/* global options ------------------------------------------------------------*/

static int toinact = 10000;          /* inactive timeout (ms) */
static int ticonnect = 10000;        /* interval to re-connect (ms) */
static int tirate = 1000;            /* avraging time for data rate (ms) */
static int buffsize = 32768;         /* receive/send buffer size (bytes) */
static char localdir[1024] = "";     /* local directory for ftp/http */
static char proxyaddr[256] = "";     /* http/ntrip/ftp proxy address */
static unsigned int tick_master = 0; /* time tick master for replay */
static int fswapmargin = 30;         /* file swap margin (s) */


/* open serial ---------------------------------------------------------------*/
serial_t *openserial(const char *path, int mode, char *msg)
{
    const int br[] = {
        300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400};
    serial_t *serial;
    int i;
    int brate = 9600;
    int bsize = 8;
    int stopb = 1;
    char *p;
    char parity = 'N';
    char dev[128];
    char port[128];
    char fctr[64] = "";

    const speed_t bs[] = {
        B300, B600, B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400};
    struct termios ios
    {
    };
    int rw = 0;
    tracet(3, "openserial: path=%s mode=%d\n", path, mode);

    if (!(serial = static_cast<serial_t *>(malloc(sizeof(serial_t)))))
        {
            return nullptr;
        }

    if ((p = strchr(const_cast<char *>(path), ':')))
        {
            strncpy(port, path, p - path);
            port[p - path] = '\0';
            sscanf(p, ":%d:%d:%c:%d:%s", &brate, &bsize, &parity, &stopb, fctr);
        }
    else if (strlen(path) < 128)
        {
            std::strncpy(port, path, 128);
            port[127] = '\0';
        }

    for (i = 0; i < 10; i++)
        {
            if (br[i] == brate)
                {
                    break;
                }
        }
    if (i >= 11)
        {
            std::snprintf(msg, MAXSTRMSG, "bitrate error (%d)", brate);
            tracet(1, "openserial: %s path=%s\n", msg, path);
            free(serial);
            return nullptr;
        }
    parity = static_cast<char>(toupper(static_cast<int>(parity)));

    std::string s_aux = "/dev/"s + std::string(port);
    s_aux.resize(128, '\0');
    int n = s_aux.length();
    for (i = 0; i < n; i++)
        {
            dev[i] = s_aux[i];
        }
    if (n == 0)
        {
            dev[0] = '\0';
        }

    if ((mode & STR_MODE_R) && (mode & STR_MODE_W))
        {
            rw = O_RDWR;
        }
    else if (mode & STR_MODE_R)
        {
            rw = O_RDONLY;
        }
    else if (mode & STR_MODE_W)
        {
            rw = O_WRONLY;
        }

    if ((serial->dev = open(dev, rw | O_NOCTTY | O_NONBLOCK)) < 0)
        {
            std::snprintf(msg, MAXSTRMSG, "device open error (%d)", errno);
            tracet(1, "openserial: %s dev=%s\n", msg, dev);
            free(serial);
            return nullptr;
        }
    tcgetattr(serial->dev, &ios);
    ios.c_iflag = 0;
    ios.c_oflag = 0;
    ios.c_lflag = 0;    /* non-canonical */
    ios.c_cc[VMIN] = 0; /* non-block-mode */
    ios.c_cc[VTIME] = 0;
    cfsetospeed(&ios, bs[i]);
    cfsetispeed(&ios, bs[i]);
    ios.c_cflag |= bsize == 7 ? CS7 : CS8;
    ios.c_cflag |= parity == 'O' ? (PARENB | PARODD) : (parity == 'E' ? PARENB : 0);
    ios.c_cflag |= stopb == 2 ? CSTOPB : 0;
    ios.c_cflag |= !strcmp(fctr, "rts") ? CRTSCTS : 0;
    tcsetattr(serial->dev, TCSANOW, &ios);
    tcflush(serial->dev, TCIOFLUSH);
    return serial;
}


/* close serial --------------------------------------------------------------*/
void closeserial(serial_t *serial)
{
    if (!serial)
        {
            return;
        }
    tracet(3, "closeserial: dev=%d\n", serial->dev);
    close(serial->dev);
    free(serial);
}


/* read serial ---------------------------------------------------------------*/
int readserial(serial_t *serial, unsigned char *buff, int n, char *msg __attribute__((unused)))
{
    int nr;
    if (!serial)
        {
            return 0;
        }
    tracet(4, "readserial: dev=%d n=%d\n", serial->dev, n);
    if ((nr = read(serial->dev, buff, n)) < 0)
        {
            return 0;
        }
    tracet(5, "readserial: exit dev=%d nr=%d\n", serial->dev, nr);
    return nr;
}


/* write serial --------------------------------------------------------------*/
int writeserial(serial_t *serial, unsigned char *buff, int n, char *msg __attribute__((unused)))
{
    int ns;
    if (!serial)
        {
            return 0;
        }
    tracet(3, "writeserial: dev=%d n=%d\n", serial->dev, n);
    if ((ns = write(serial->dev, buff, n)) < 0)
        {
            return 0;
        }
    tracet(5, "writeserial: exit dev=%d ns=%d\n", serial->dev, ns);
    return ns;
}


/* get state serial ----------------------------------------------------------*/
int stateserial(serial_t *serial)
{
    return !serial ? 0 : (serial->error ? -1 : 2);
}


/* open file -----------------------------------------------------------------*/
int openfile_(file_t *file, gtime_t time, char *msg)
{
    FILE *fp;
    char tagpath[MAXSTRPATH + 4] = "";
    char tagh[TIMETAGH_LEN + 1] = "";

    tracet(3, "openfile_: path=%s time=%s\n", file->path.data(), time_str(time, 0));

    file->time = utc2gpst(timeget());
    file->tick = file->tick_f = tickget();
    file->fpos = 0;

    /* use stdin or stdout if file path is empty */
    if (file->path.empty())
        {
            file->fp = file->mode & STR_MODE_R ? stdin : stdout;
            return 1;
        }
    /* replace keywords */
    reppath(file->path, file->openpath, time, "", "");

    /* create directory */
    if ((file->mode & STR_MODE_W) && !(file->mode & STR_MODE_R))
        {
            createdir(file->openpath.data());
        }

    char const *mode;
    if (file->mode & STR_MODE_R)
        {
            mode = "rb";
        }
    else
        {
            mode = "wb";
        }

    if (!(file->fp = fopen(file->openpath.data(), mode)))
        {
            std::snprintf(msg, MAXSTRMSG, "file open error");
            tracet(1, "openfile: %s\n", msg);
            return 0;
        }
    tracet(4, "openfile_: open file %s (%s)\n", file->openpath.data(), mode);

    std::snprintf(tagpath, MAXSTRPATH + 4, "%s.tag", file->openpath.data());

    if (file->timetag)
        { /* output/sync time-tag */
            if (!(file->fp_tag = fopen(tagpath, mode)))
                {
                    std::snprintf(msg, MAXSTRMSG, "tag open error");
                    tracet(1, "openfile: %s\n", msg);
                    fclose(file->fp);
                    return 0;
                }
            tracet(4, "openfile_: open tag file %s (%s)\n", tagpath, mode);

            if (file->mode & STR_MODE_R)
                {
                    if (fread(&tagh, TIMETAGH_LEN, 1, file->fp_tag) == 1 &&
                        fread(&file->time, sizeof(file->time), 1, file->fp_tag) == 1)
                        {
                            memcpy(&file->tick_f, tagh + TIMETAGH_LEN - 4, sizeof(file->tick_f));
                        }
                    else
                        {
                            file->tick_f = 0;
                        }
                    /* adust time to read playback file */
                    timeset(file->time);
                }
            else
                {
                    std::snprintf(tagh, TIMETAGH_LEN + 1, "TIMETAG RTKLIB %s", VER_RTKLIB);
                    memcpy(tagh + TIMETAGH_LEN - 4, &file->tick_f, sizeof(file->tick_f));
                    fwrite(&tagh, 1, TIMETAGH_LEN, file->fp_tag);
                    fwrite(&file->time, 1, sizeof(file->time), file->fp_tag);
                    /* time tag file structure   */
                    /*   HEADER(60)+TICK(4)+TIME(12)+ */
                    /*   TICK0(4)+FPOS0(4/8)+    */
                    /*   TICK1(4)+FPOS1(4/8)+... */
                }
        }
    else if (file->mode & STR_MODE_W)
        { /* remove time-tag */
            if ((fp = fopen(tagpath, "rbe")))
                {
                    fclose(fp);
                    if (remove(tagpath) != 0)
                        {
                            trace(1, "Error removing file");
                        }
                }
        }
    return 1;
}


/* close file ----------------------------------------------------------------*/
void closefile_(file_t *file)
{
    tracet(3, "closefile_: path=%s\n", file->path.data());
    if (file->fp)
        {
            fclose(file->fp);
            file->fp = nullptr;
        }
    if (file->fp_tag)
        {
            fclose(file->fp_tag);
            file->fp_tag = nullptr;
        }
    if (file->fp_tmp)
        {
            fclose(file->fp_tmp);
            file->fp_tmp = nullptr;
        }
    if (file->fp_tag_tmp)
        {
            fclose(file->fp_tag_tmp);
            file->fp_tag_tmp = nullptr;
        }
}


/* open file (path=filepath[::T[::+<off>][::x<speed>]][::S=swapintv]) --------*/
file_t *openfile(std::string const &path, int mode, char *msg)
{
    tracet(3, "openfile: path=%s mode=%d\n", path.data(), mode);

    if ((mode & (STR_MODE_R | STR_MODE_W)) == 0)
        {
            return nullptr;
        }

    // Split the string by regular expression (in this case, the trivial "::" string)
    std::regex re("::");
    auto first = std::sregex_token_iterator(path.begin(), path.end(), re, -1);
    auto last = std::sregex_token_iterator();
    std::deque<std::string> tokens(first, last);

    auto file = std::make_unique<file_t>();

    file->mode = mode;
    file->path = tokens.front();  // first token is the path
    tokens.pop_front();


    /* file options */
    while (not tokens.empty())
        {
            auto tag = tokens.front();
            tokens.pop_front();

            // edge case that may not be possible, but I don't want to test for right now
            if (tag.empty()) continue;  // NOLINT(readability-braces-around-statements)

            if (tag == "T")
                {
                    file->timetag = 1;
                }
            else if (tag[0] == '+')
                {
                    double start = 0.0;
                    std::istringstream ss(tag);
                    ss.ignore(1, '+') >> start;
                    // do we care if there are extra characters?

                    if (start < 0)
                        {
                            start = 0;
                        }
                    file->start = start;
                }
            else if (tag[0] == 'x')
                {
                    double speed = 0.0;
                    std::istringstream ss(tag);
                    ss.ignore(1, 'x') >> speed;
                    // do we care if there are extra characters?
                    file->speed = speed;
                }
            else if (tag[0] == 'S')
                {
                    double swapintv = 0.0;
                    std::istringstream ss(tag);
                    ss.ignore(1, 'S').ignore(1, '=') >> swapintv;
                    // do we care if there are extra characters?
                    if (swapintv < 0) swapintv = 0;  // NOLINT(readability-braces-around-statements)
                    file->swapintv = swapintv;
                }
            else
                {
                    // unexpected value ... not previously handled
                }
        }

    initlock(&file->lock);

    auto time = utc2gpst(timeget());

    /* open new file */
    if (!openfile_(file.get(), time, msg))
        {
            file.reset();
        }
    return file.release();  // ownership belongs to the caller now
}


/* close file ----------------------------------------------------------------*/
void closefile(file_t *file)
{
    if (file)
        {
            std::unique_ptr<file_t> fileH(file);
            tracet(3, "closefile: fp=%p \n", fileH->fp);

            closefile_(fileH.get());
        }
}


/* open new swap file --------------------------------------------------------*/
void swapfile(file_t *file, gtime_t time, char *msg)
{
    std::string openpath;

    tracet(3, "swapfile: fp=%p \n time=%s\n", file->fp, time_str(time, 0));

    /* return if old swap file open */
    if (file->fp_tmp || file->fp_tag_tmp)
        {
            return;
        }

    /* check path of new swap file */
    reppath(file->path, openpath, time, "", "");

    if (openpath == file->openpath)
        {
            tracet(2, "swapfile: no need to swap %s\n", openpath.data());
            return;
        }
    /* save file pointer to temporary pointer */
    file->fp_tmp = file->fp;
    file->fp_tag_tmp = file->fp_tag;

    /* open new swap file */
    openfile_(file, time, msg);
}


/* close old swap file -------------------------------------------------------*/
void swapclose(file_t *file)
{
    tracet(3, "swapclose: fp_tmp=%p \n", file->fp_tmp);
    if (file->fp_tmp)
        {
            fclose(file->fp_tmp);
        }
    if (file->fp_tag_tmp)
        {
            fclose(file->fp_tag_tmp);
        }
    file->fp_tmp = file->fp_tag_tmp = nullptr;
}


/* get state file ------------------------------------------------------------*/
int statefile(file_t *file)
{
    return file ? 2 : 0;
}


/* read file -----------------------------------------------------------------*/
int readfile(file_t *file, unsigned char *buff, int nmax, char *msg)
{
    struct timeval tv = {0, 0};
    fd_set rs;
    unsigned int t;
    unsigned int tick;
    int nr = 0;
    size_t fpos;

    if (!file)
        {
            return 0;
        }
    tracet(4, "readfile: fp=%p nmax=%d\n", file->fp, nmax);

    if (file->fp == stdin)
        {
            /* input from stdin */
            std::memset(&rs, 0, sizeof(fd_set));
            FD_SET(0, &rs);
            if (!select(1, &rs, nullptr, nullptr, &tv))
                {
                    return 0;
                }
            if ((nr = read(0, buff, nmax)) < 0)
                {
                    return 0;
                }
            return nr;
        }
    if (file->fp_tag)
        {
            if (file->repmode)
                { /* slave */
                    t = (tick_master + file->offset);
                }
            else
                { /* master */
                    t = static_cast<unsigned int>((tickget() - file->tick) * file->speed + file->start * 1000.0);
                }
            for (;;)
                { /* seek file position */
                    if (fread(&tick, sizeof(tick), 1, file->fp_tag) < 1 ||
                        fread(&fpos, sizeof(fpos), 1, file->fp_tag) < 1)
                        {
                            if (fseek(file->fp, 0, SEEK_END) != 0)
                                {
                                    trace(1, "fseek error");
                                }
                            std::snprintf(msg, MAXSTRPATH, "end");
                            break;
                        }
                    if (file->repmode || file->speed > 0.0)
                        {
                            if (static_cast<int>(tick - t) < 1)
                                {
                                    continue;
                                }
                        }
                    if (!file->repmode)
                        {
                            tick_master = tick;
                        }

                    std::snprintf(msg, MAXSTRPATH, "T%+.1fs", static_cast<int>(tick) < 0 ? 0.0 : static_cast<int>(tick) / 1000.0);

                    if (static_cast<int>(fpos - file->fpos) >= nmax)
                        {
                            if (fseek(file->fp, fpos, SEEK_SET) != 0)
                                {
                                    trace(1, "Error fseek");
                                }
                            file->fpos = fpos;
                            return 0;
                        }
                    nmax = static_cast<int>(fpos - file->fpos);

                    if (file->repmode || file->speed > 0.0)
                        {
                            if (fseek(file->fp_tag, -static_cast<int64_t>(sizeof(tick) + sizeof(fpos)), SEEK_CUR) != 0)
                                {
                                    trace(1, "Error fseek");
                                }
                        }
                    break;
                }
        }
    if (nmax > 0)
        {
            nr = fread(buff, 1, nmax, file->fp);
            file->fpos += nr;
            if (nr <= 0)
                {
                    std::snprintf(msg, MAXSTRPATH, "end");
                }
        }
    tracet(5, "readfile: fp=%p \n nr=%d fpos=%u\n", file->fp, nr, file->fpos);
    return nr;
}


/* write file ----------------------------------------------------------------*/
int writefile(file_t *file, unsigned char *buff, int n, char *msg)
{
    gtime_t wtime;
    unsigned int ns;
    unsigned int tick = tickget();
    int week1;
    int week2;
    double tow1;
    double tow2;
    double intv;
    size_t fpos;
    size_t fpos_tmp;

    if (!file)
        {
            return 0;
        }
    tracet(3, "writefile: fp=%p \n n=%d\n", file->fp, n);

    wtime = utc2gpst(timeget()); /* write time in gpst */

    /* swap writing file */
    if (file->swapintv > 0.0 && file->wtime.time != 0)
        {
            intv = file->swapintv * 3600.0;
            tow1 = time2gpst(file->wtime, &week1);
            tow2 = time2gpst(wtime, &week2);
            tow2 += 604800.0 * (week2 - week1);

            /* open new swap file */
            if (floor((tow1 + fswapmargin) / intv) < floor((tow2 + fswapmargin) / intv))
                {
                    swapfile(file, timeadd(wtime, fswapmargin), msg);
                }
            /* close old swap file */
            if (floor((tow1 - fswapmargin) / intv) < floor((tow2 - fswapmargin) / intv))
                {
                    swapclose(file);
                }
        }
    if (!file->fp)
        {
            return 0;
        }

    ns = fwrite(buff, 1, n, file->fp);
    fpos = ftell(file->fp);
    fflush(file->fp);
    file->wtime = wtime;

    if (file->fp_tmp)
        {
            fwrite(buff, 1, n, file->fp_tmp);
            fpos_tmp = ftell(file->fp_tmp);
            fflush(file->fp_tmp);
        }
    if (file->fp_tag)
        {
            tick -= file->tick;
            fwrite(&tick, 1, sizeof(tick), file->fp_tag);
            fwrite(&fpos, 1, sizeof(fpos), file->fp_tag);
            fflush(file->fp_tag);

            if (file->fp_tag_tmp)
                {
                    fwrite(&tick, 1, sizeof(tick), file->fp_tag_tmp);
                    fwrite(&fpos_tmp, 1, sizeof(fpos_tmp), file->fp_tag_tmp);
                    fflush(file->fp_tag_tmp);
                }
        }
    tracet(5, "writefile: fp=%p \n ns=%d tick=%5d fpos=%zd\n", file->fp, ns, tick, fpos);

    return static_cast<int>(ns);
}


/* sync files by time-tag ----------------------------------------------------*/
void syncfile(file_t *file1, file_t *file2)
{
    if (!file1->fp_tag || !file2->fp_tag)
        {
            return;
        }
    file1->repmode = 0;
    file2->repmode = 1;
    file2->offset = static_cast<int>(file1->tick_f - file2->tick_f);
}


/* decode tcp/ntrip path (path=[user[:passwd]@]addr[:port][/mntpnt[:str]]) ---*/
void decodetcppath(const char *path, char *addr, char *port, char *user,
    char *passwd, char *mntpnt, char *str)
{
    char buff[MAXSTRPATH] = "";
    char *p;
    char *q;

    tracet(4, "decodetcpepath: path=%s\n", path);

    if (port)
        {
            *port = '\0';
        }
    if (user)
        {
            *user = '\0';
        }
    if (passwd)
        {
            *passwd = '\0';
        }
    if (mntpnt)
        {
            *mntpnt = '\0';
        }
    if (str)
        {
            *str = '\0';
        }

    if (strlen(path) < MAXSTRPATH)
        {
            std::strncpy(buff, path, MAXSTRPATH);
            buff[MAXSTRPATH - 1] = '\0';
        }

    if (!(p = strrchr(buff, '@')))
        {
            p = buff;
        }

    if ((p = strchr(p, '/')))
        {
            if ((q = strchr(p + 1, ':')))
                {
                    *q = '\0';
                    if (str)
                        {
                            std::strncpy(str, q + 1, NTRIP_MAXSTR);
                            str[NTRIP_MAXSTR - 1] = '\0';
                        }
                }
            *p = '\0';
            if (mntpnt)
                {
                    std::strncpy(mntpnt, p + 1, 256);
                    mntpnt[255] = '\0';
                }
        }
    if ((p = strrchr(buff, '@')))
        {
            *p++ = '\0';
            if ((q = strchr(buff, ':')))
                {
                    *q = '\0';
                    if (passwd)
                        {
                            std::strncpy(passwd, q + 1, 256);
                            passwd[255] = '\0';
                        }
                }
            if (user)
                {
                    std::memcpy(user, buff, 256);
                    user[255] = '\0';
                }
        }
    else
        {
            p = buff;
        }

    if ((q = strchr(p, ':')))
        {
            *q = '\0';
            if (port)
                {
                    std::strncpy(port, q + 1, 256);
                    port[255] = '\0';
                }
        }
    if (addr)
        {
            int ret = std::snprintf(addr, 256, "%s", p);  // NOLINT(runtime/printf)
            if (ret < 0 || ret >= 256)
                {
                    tracet(1, "error reading address");
                }
        }
}


/* get socket error ----------------------------------------------------------*/
int errsock() { return errno; }


/* set socket option ---------------------------------------------------------*/
int setsock(socket_t sock, char *msg)
{
    int bs = buffsize;
    int mode = 1;
    struct timeval tv = {0, 0};

    tracet(3, "setsock: sock=%d\n", sock);

    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&tv), sizeof(tv)) == -1 ||
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char *>(&tv), sizeof(tv)) == -1)
        {
            std::snprintf(msg, MAXSTRMSG, "sockopt error: notimeo");
            tracet(1, "setsock: setsockopt error 1 sock=%d err=%d\n", sock, errsock());
            closesocket(sock);
            return 0;
        }
    if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char *>(&bs), sizeof(bs)) == -1 ||
        setsockopt(sock, SOL_SOCKET, SO_SNDBUF, reinterpret_cast<const char *>(&bs), sizeof(bs)) == -1)
        {
            tracet(1, "setsock: setsockopt error 2 sock=%d err=%d bs=%d\n", sock, errsock(), bs);
            std::snprintf(msg, MAXSTRMSG, "sockopt error: bufsiz");
        }
    if (setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&mode), sizeof(mode)) == -1)
        {
            tracet(1, "setsock: setsockopt error 3 sock=%d err=%d\n", sock, errsock());
            std::snprintf(msg, MAXSTRMSG, "sockopt error: nodelay");
        }
    return 1;
}


/* non-block accept ----------------------------------------------------------*/
socket_t accept_nb(socket_t sock, struct sockaddr *addr, socklen_t *len)
{
    struct timeval tv = {0, 0};
    fd_set rs;
    std::memset(&rs, 0, sizeof(fd_set));
    FD_SET(sock, &rs);
    if (!select(sock + 1, &rs, nullptr, nullptr, &tv))
        {
            return 0;
        }
    return accept(sock, addr, len);
}


/* non-block connect ---------------------------------------------------------*/
int connect_nb(socket_t sock, struct sockaddr *addr, socklen_t len)
{
    struct timeval tv = {0, 0};
    fd_set rs;
    fd_set ws;
    int err;
    int flag;

    flag = fcntl(sock, F_GETFL, 0);
    if (fcntl(sock, F_SETFL, flag | O_NONBLOCK) == -1)
        {
            trace(1, "fcntl error");
        }
    if (connect(sock, addr, len) == -1)
        {
            err = errsock();
            if (err != EISCONN && err != EINPROGRESS && err != EALREADY)
                {
                    return -1;
                }
            std::memset(&rs, 0, sizeof(fd_set));
            FD_SET(sock, &rs);
            ws = rs;
            if (select(sock + 1, &rs, &ws, nullptr, &tv) == 0)
                {
                    return 0;
                }
        }
    return 1;
}


/* non-block receive ---------------------------------------------------------*/
int recv_nb(socket_t sock, unsigned char *buff, int n)
{
    struct timeval tv = {0, 0};
    fd_set rs;
    std::memset(&rs, 0, sizeof(fd_set));
    FD_SET(sock, &rs);
    if (!select(sock + 1, &rs, nullptr, nullptr, &tv))
        {
            return 0;
        }
    return recv(sock, reinterpret_cast<char *>(buff), n, 0);
}


/* non-block send ------------------------------------------------------------*/
int send_nb(socket_t sock, unsigned char *buff, int n)
{
    struct timeval tv = {0, 0};
    fd_set ws;
    std::memset(&ws, 0, sizeof(fd_set));
    FD_SET(sock, &ws);
    if (!select(sock + 1, nullptr, &ws, nullptr, &tv))
        {
            return 0;
        }
    return send(sock, reinterpret_cast<char *>(buff), n, 0);
}


/* generate tcp socket -------------------------------------------------------*/
int gentcp(tcp_t *tcp, int type, char *msg)
{
    struct hostent *hp;
#ifdef SVR_REUSEADDR
    int opt = 1;
#endif

    tracet(3, "gentcp: type=%d\n", type);

    /* generate socket */
    if ((tcp->sock = socket(AF_INET, SOCK_STREAM, 0)) == -1)
        {
            std::snprintf(msg, MAXSTRMSG, "socket error (%d)", errsock());
            tracet(1, "gentcp: socket error err=%d\n", errsock());
            tcp->state = -1;
            return 0;
        }
    if (!setsock(tcp->sock, msg))
        {
            tcp->state = -1;
            return 0;
        }
    memset(&tcp->addr, 0, sizeof(tcp->addr));
    tcp->addr.sin_family = AF_INET;
    tcp->addr.sin_port = htons(tcp->port);

    if (type == 0)
        { /* server socket */
#ifdef SVR_REUSEADDR
            /* multiple-use of server socket */
            setsockopt(tcp->sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt,
                sizeof(opt));
#endif
            if (bind(tcp->sock, reinterpret_cast<struct sockaddr *>(&tcp->addr), sizeof(tcp->addr)) == -1)
                {
                    std::snprintf(msg, MAXSTRMSG, "bind error (%d) : %d", errsock(), tcp->port);
                    tracet(1, "gentcp: bind error port=%d err=%d\n", tcp->port, errsock());
                    closesocket(tcp->sock);
                    tcp->state = -1;
                    return 0;
                }
            listen(tcp->sock, 5);
        }
    else
        { /* client socket */
            if (!(hp = gethostbyname(tcp->saddr)))
                {
                    std::snprintf(msg, MAXSTRMSG, "address error (%s)", tcp->saddr);
                    tracet(1, "gentcp: gethostbyname error addr=%s err=%d\n", tcp->saddr, errsock());
                    closesocket(tcp->sock);
                    tcp->state = 0;
                    tcp->tcon = ticonnect;
                    tcp->tdis = tickget();
                    return 0;
                }
            memcpy(&tcp->addr.sin_addr, hp->h_addr, hp->h_length);
        }
    tcp->state = 1;
    tcp->tact = tickget();
    tracet(5, "gentcp: exit sock=%d\n", tcp->sock);
    return 1;
}


/* disconnect tcp ------------------------------------------------------------*/
void discontcp(tcp_t *tcp, int tcon)
{
    tracet(3, "discontcp: sock=%d tcon=%d\n", tcp->sock, tcon);
    closesocket(tcp->sock);
    tcp->state = 0;
    tcp->tcon = tcon;
    tcp->tdis = tickget();
}


/* open tcp server -----------------------------------------------------------*/
tcpsvr_t *opentcpsvr(const char *path, char *msg)
{
    tcpsvr_t *tcpsvr;
    tcpsvr_t tcpsvr0{};
    char port[256] = "";
    tracet(3, "opentcpsvr: path=%s\n", path);

    if (!(tcpsvr = static_cast<tcpsvr_t *>(malloc(sizeof(tcpsvr_t)))))
        {
            return nullptr;
        }
    *tcpsvr = tcpsvr0;
    decodetcppath(path, tcpsvr->svr.saddr, port, nullptr, nullptr, nullptr, nullptr);
    if (sscanf(port, "%d", &tcpsvr->svr.port) < 1)
        {
            std::snprintf(msg, MAXSTRMSG, "port error: %s", port);
            tracet(1, "opentcpsvr: port error port=%s\n", port);
            free(tcpsvr);
            return nullptr;
        }
    if (!gentcp(&tcpsvr->svr, 0, msg))
        {
            free(tcpsvr);
            return nullptr;
        }
    tcpsvr->svr.tcon = 0;
    return tcpsvr;
}


/* close tcp server ----------------------------------------------------------*/
void closetcpsvr(tcpsvr_t *tcpsvr)
{
    int i;
    tracet(3, "closetcpsvr:\n");
    for (i = 0; i < MAXCLI; i++)
        {
            if (tcpsvr->cli[i].state)
                {
                    closesocket(tcpsvr->cli[i].sock);
                }
        }
    closesocket(tcpsvr->svr.sock);
    free(tcpsvr);
}


/* update tcp server ---------------------------------------------------------*/
void updatetcpsvr(tcpsvr_t *tcpsvr, char *msg)
{
    char saddr[256] = "";
    int i;
    int j;
    int n = 0;

    tracet(3, "updatetcpsvr: state=%d\n", tcpsvr->svr.state);

    if (tcpsvr->svr.state == 0)
        {
            return;
        }

    for (i = 0; i < MAXCLI; i++)
        {
            if (tcpsvr->cli[i].state)
                {
                    continue;
                }
            for (j = i + 1; j < MAXCLI; j++)
                {
                    if (!tcpsvr->cli[j].state)
                        {
                            continue;
                        }
                    tcpsvr->cli[i] = tcpsvr->cli[j];
                    tcpsvr->cli[j].state = 0;
                    break;
                }
        }
    for (i = 0; i < MAXCLI; i++)
        {
            if (!tcpsvr->cli[i].state)
                {
                    continue;
                }
            std::snprintf(saddr, sizeof(saddr), "%s", tcpsvr->cli[i].saddr);
            n++;
        }
    if (n == 0)
        {
            tcpsvr->svr.state = 1;
            std::snprintf(msg, MAXSTRMSG, "waiting...");
            return;
        }
    tcpsvr->svr.state = 2;
    if (n == 1)
        {
            std::snprintf(msg, MAXSTRMSG, "%s", saddr);
        }
    else
        {
            std::snprintf(msg, MAXSTRMSG, "%d clients", n);
        }
}


/* accept client connection --------------------------------------------------*/
int accsock(tcpsvr_t *tcpsvr, char *msg)
{
    struct sockaddr_in addr
    {
    };
    socket_t sock;
    socklen_t len = sizeof(addr);
    int i;
    int err;

    tracet(3, "accsock: sock=%d\n", tcpsvr->svr.sock);

    for (i = 0; i < MAXCLI; i++)
        {
            if (tcpsvr->cli[i].state == 0)
                {
                    break;
                }
        }
    if (i >= MAXCLI)
        {
            return 0; /* too many client */
        }

    if ((sock = accept_nb(tcpsvr->svr.sock, reinterpret_cast<struct sockaddr *>(&addr), &len)) == -1)
        {
            err = errsock();
            std::snprintf(msg, MAXSTRMSG, "accept error (%d)", err);
            tracet(1, "accsock: accept error sock=%d err=%d\n", tcpsvr->svr.sock, err);
            closesocket(tcpsvr->svr.sock);
            tcpsvr->svr.state = 0;
            return 0;
        }
    if (sock == 0)
        {
            return 0;
        }

    tcpsvr->cli[i].sock = sock;
    if (!setsock(tcpsvr->cli[i].sock, msg))
        {
            return 0;
        }
    memcpy(&tcpsvr->cli[i].addr, &addr, sizeof(addr));
    if (strlen(inet_ntoa(addr.sin_addr)) < 256)
        {
            std::strncpy(tcpsvr->cli[i].saddr, inet_ntoa(addr.sin_addr), 256);
            tcpsvr->cli[i].saddr[255] = '\0';
        }
    std::snprintf(msg, MAXSTRMSG, "%s", tcpsvr->cli[i].saddr);
    tracet(2, "accsock: connected sock=%d addr=%s\n", tcpsvr->cli[i].sock, tcpsvr->cli[i].saddr);
    tcpsvr->cli[i].state = 2;
    tcpsvr->cli[i].tact = tickget();
    return 1;
}


/* wait socket accept --------------------------------------------------------*/
int waittcpsvr(tcpsvr_t *tcpsvr, char *msg)
{
    tracet(4, "waittcpsvr: sock=%d state=%d\n", tcpsvr->svr.sock, tcpsvr->svr.state);
    if (tcpsvr->svr.state <= 0)
        {
            return 0;
        }
    while (accsock(tcpsvr, msg))
        {
        }
    updatetcpsvr(tcpsvr, msg);
    return tcpsvr->svr.state == 2;
}


/* read tcp server -----------------------------------------------------------*/
int readtcpsvr(tcpsvr_t *tcpsvr, unsigned char *buff, int n, char *msg)
{
    int nr;
    int err;

    tracet(4, "readtcpsvr: state=%d n=%d\n", tcpsvr->svr.state, n);

    if (!waittcpsvr(tcpsvr, msg) || tcpsvr->cli[0].state != 2)
        {
            return 0;
        }

    if ((nr = recv_nb(tcpsvr->cli[0].sock, buff, n)) == -1)
        {
            err = errsock();
            tracet(1, "readtcpsvr: recv error sock=%d err=%d\n", tcpsvr->cli[0].sock, err);
            std::snprintf(msg, MAXSTRMSG, "recv error (%d)", err);
            discontcp(&tcpsvr->cli[0], ticonnect);
            updatetcpsvr(tcpsvr, msg);
            return 0;
        }
    if (nr > 0)
        {
            tcpsvr->cli[0].tact = tickget();
        }
    tracet(5, "readtcpsvr: exit sock=%d nr=%d\n", tcpsvr->cli[0].sock, nr);
    return nr;
}


/* write tcp server ----------------------------------------------------------*/
int writetcpsvr(tcpsvr_t *tcpsvr, unsigned char *buff, int n, char *msg)
{
    int i;
    int ns = 0;
    int err;

    tracet(3, "writetcpsvr: state=%d n=%d\n", tcpsvr->svr.state, n);

    if (!waittcpsvr(tcpsvr, msg))
        {
            return 0;
        }

    for (i = 0; i < MAXCLI; i++)
        {
            if (tcpsvr->cli[i].state != 2)
                {
                    continue;
                }

            if ((ns = send_nb(tcpsvr->cli[i].sock, buff, n)) == -1)
                {
                    err = errsock();
                    tracet(1, "writetcpsvr: send error i=%d sock=%d err=%d\n", i, tcpsvr->cli[i].sock, err);
                    std::snprintf(msg, MAXSTRMSG, "send error (%d)", err);
                    discontcp(&tcpsvr->cli[i], ticonnect);
                    updatetcpsvr(tcpsvr, msg);
                    return 0;
                }
            if (ns > 0)
                {
                    tcpsvr->cli[i].tact = tickget();
                }
            tracet(5, "writetcpsvr: send i=%d ns=%d\n", i, ns);
        }
    return ns;
}


/* get state tcp server ------------------------------------------------------*/
int statetcpsvr(tcpsvr_t *tcpsvr)
{
    return tcpsvr ? tcpsvr->svr.state : 0;
}


/* connect server ------------------------------------------------------------*/
int consock(tcpcli_t *tcpcli, char *msg)
{
    int stat;
    int err;

    tracet(3, "consock: sock=%d\n", tcpcli->svr.sock);

    /* wait re-connect */
    if (tcpcli->svr.tcon < 0 || (tcpcli->svr.tcon > 0 &&
                                    static_cast<int>(tickget() - tcpcli->svr.tdis) < tcpcli->svr.tcon))
        {
            return 0;
        }
    /* non-block connect */
    if ((stat = connect_nb(tcpcli->svr.sock, reinterpret_cast<struct sockaddr *>(&tcpcli->svr.addr),
             sizeof(tcpcli->svr.addr))) == -1)
        {
            err = errsock();
            std::snprintf(msg, MAXSTRMSG, "connect error (%d)", err);
            tracet(1, "consock: connect error sock=%d err=%d\n", tcpcli->svr.sock, err);
            closesocket(tcpcli->svr.sock);
            tcpcli->svr.state = 0;
            return 0;
        }
    if (!stat)
        { /* not connect */
            std::snprintf(msg, MAXSTRMSG, "connecting...");
            return 0;
        }
    std::snprintf(msg, MAXSTRMSG, "%s", tcpcli->svr.saddr);
    tracet(2, "consock: connected sock=%d addr=%s\n", tcpcli->svr.sock, tcpcli->svr.saddr);
    tcpcli->svr.state = 2;
    tcpcli->svr.tact = tickget();
    return 1;
}


/* open tcp client -----------------------------------------------------------*/
tcpcli_t *opentcpcli(const char *path, char *msg)
{
    tcpcli_t *tcpcli;
    tcpcli_t tcpcli0{};
    char port[256] = "";

    tracet(3, "opentcpcli: path=%s\n", path);

    if (!(tcpcli = static_cast<tcpcli_t *>(malloc(sizeof(tcpcli_t)))))
        {
            return nullptr;
        }
    *tcpcli = tcpcli0;
    decodetcppath(path, tcpcli->svr.saddr, port, nullptr, nullptr, nullptr, nullptr);
    if (sscanf(port, "%d", &tcpcli->svr.port) < 1)
        {
            std::snprintf(msg, MAXSTRMSG, "port error: %s", port);
            tracet(1, "opentcp: port error port=%s\n", port);
            free(tcpcli);
            return nullptr;
        }
    tcpcli->svr.tcon = 0;
    tcpcli->toinact = toinact;
    tcpcli->tirecon = ticonnect;
    return tcpcli;
}


/* close tcp client ----------------------------------------------------------*/
void closetcpcli(tcpcli_t *tcpcli)
{
    tracet(3, "closetcpcli: sock=%d\n", tcpcli->svr.sock);
    closesocket(tcpcli->svr.sock);
    free(tcpcli);
}


/* wait socket connect -------------------------------------------------------*/
int waittcpcli(tcpcli_t *tcpcli, char *msg)
{
    tracet(4, "waittcpcli: sock=%d state=%d\n", tcpcli->svr.sock, tcpcli->svr.state);

    if (tcpcli->svr.state < 0)
        {
            return 0;
        }

    if (tcpcli->svr.state == 0)
        { /* close */
            if (!gentcp(&tcpcli->svr, 1, msg))
                {
                    return 0;
                }
        }
    if (tcpcli->svr.state == 1)
        { /* wait */
            if (!consock(tcpcli, msg))
                {
                    return 0;
                }
        }
    if (tcpcli->svr.state == 2)
        { /* connect */
            if (tcpcli->toinact > 0 &&
                static_cast<int>(tickget() - tcpcli->svr.tact) > tcpcli->toinact)
                {
                    std::snprintf(msg, MAXSTRMSG, "timeout");
                    tracet(2, "waittcpcli: inactive timeout sock=%d\n", tcpcli->svr.sock);
                    discontcp(&tcpcli->svr, tcpcli->tirecon);
                    return 0;
                }
        }
    return 1;
}


/* read tcp client -----------------------------------------------------------*/
int readtcpcli(tcpcli_t *tcpcli, unsigned char *buff, int n, char *msg)
{
    int nr;
    int err;

    tracet(4, "readtcpcli: sock=%d state=%d n=%d\n", tcpcli->svr.sock, tcpcli->svr.state, n);

    if (!waittcpcli(tcpcli, msg))
        {
            return 0;
        }

    if ((nr = recv_nb(tcpcli->svr.sock, buff, n)) == -1)
        {
            err = errsock();
            tracet(1, "readtcpcli: recv error sock=%d err=%d\n", tcpcli->svr.sock, err);
            std::snprintf(msg, MAXSTRMSG, "recv error (%d)", err);
            discontcp(&tcpcli->svr, tcpcli->tirecon);
            return 0;
        }
    if (nr > 0)
        {
            tcpcli->svr.tact = tickget();
        }
    tracet(5, "readtcpcli: exit sock=%d nr=%d\n", tcpcli->svr.sock, nr);
    return nr;
}


/* write tcp client ----------------------------------------------------------*/
int writetcpcli(tcpcli_t *tcpcli, unsigned char *buff, int n, char *msg)
{
    int ns;
    int err;

    tracet(3, "writetcpcli: sock=%d state=%d n=%d\n", tcpcli->svr.sock, tcpcli->svr.state, n);

    if (!waittcpcli(tcpcli, msg))
        {
            return 0;
        }

    if ((ns = send_nb(tcpcli->svr.sock, buff, n)) == -1)
        {
            err = errsock();
            tracet(1, "writetcp: send error sock=%d err=%d\n", tcpcli->svr.sock, err);
            std::snprintf(msg, MAXSTRMSG, "send error (%d)", err);
            discontcp(&tcpcli->svr, tcpcli->tirecon);
            return 0;
        }
    if (ns > 0)
        {
            tcpcli->svr.tact = tickget();
        }
    tracet(5, "writetcpcli: exit sock=%d ns=%d\n", tcpcli->svr.sock, ns);
    return ns;
}


/* get state tcp client ------------------------------------------------------*/
int statetcpcli(tcpcli_t *tcpcli)
{
    return tcpcli ? tcpcli->svr.state : 0;
}


/* base64 encoder ------------------------------------------------------------*/
int encbase64(char *str, const unsigned char *byte, int n)
{
    const char table[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int i;
    int j;
    int k;
    int b;

    tracet(4, "encbase64: n=%d\n", n);

    for (i = j = 0; i / 8 < n;)
        {
            for (k = b = 0; k < 6; k++, i++)
                {
                    b <<= 1;
                    if (i / 8 < n)
                        {
                            b |= (byte[i / 8] >> (7 - i % 8)) & 0x1;
                        }
                }
            str[j++] = table[b];
        }
    while (j & 0x3)
        {
            str[j++] = '=';
        }
    str[j] = '\0';
    tracet(5, "encbase64: str=%s\n", str);
    return j;
}


/* send ntrip server request -------------------------------------------------*/
int reqntrip_s(ntrip_t *ntrip, char *msg)
{
    char buff[256 + NTRIP_MAXSTR];
    char *p = buff;
    char *s;
    s = p;

    tracet(3, "reqntrip_s: state=%d\n", ntrip->state);

    p += std::snprintf(p, 256 + NTRIP_MAXSTR, "SOURCE %s %s\r\n", ntrip->passwd, ntrip->mntpnt);
    p += std::snprintf(p, NTRIP_MAXSTR - (p - s), "Source-Agent: NTRIP %s\r\n", NTRIP_AGENT);
    p += std::snprintf(p, NTRIP_MAXSTR - (p - s), "STR: %s\r\n", ntrip->str);
    p += std::snprintf(p, sizeof("\r\n") + 1, "\r\n");

    if (writetcpcli(ntrip->tcp, reinterpret_cast<unsigned char *>(buff), p - buff, msg) != p - buff)
        {
            return 0;
        }

    tracet(2, "reqntrip_s: send request state=%d ns=%" PRIdPTR "\n", ntrip->state, p - buff);
    tracet(5, "reqntrip_s: n=%" PRIdPTR " buff=\n%s\n", p - buff, buff);
    ntrip->state = 1;
    return 1;
}


/* send ntrip client request -------------------------------------------------*/
int reqntrip_c(ntrip_t *ntrip, char *msg)
{
    char buff[1024];
    char user[512];
    char *p = buff;
    char *s;
    s = p;

    tracet(3, "reqntrip_c: state=%d\n", ntrip->state);

    p += std::snprintf(p, NTRIP_MAXSTR, "GET %s/%s HTTP/1.0\r\n", ntrip->url, ntrip->mntpnt);
    p += std::snprintf(p, NTRIP_MAXSTR - (p - s), "User-Agent: NTRIP %s\r\n", NTRIP_AGENT);

    if (!*ntrip->user)
        {
            p += std::snprintf(p, NTRIP_MAXSTR - (p - s), "Accept: */*\r\n");
            p += std::snprintf(p, NTRIP_MAXSTR - (p - s), "Connection: close\r\n");
        }
    else
        {
            std::snprintf(user, sizeof(user), "%s:%s", ntrip->user, ntrip->passwd);
            p += std::snprintf(p, NTRIP_MAXSTR - (p - s), "Authorization: Basic ");
            p += encbase64(p, reinterpret_cast<unsigned char *>(user), strlen(user));
            p += std::snprintf(p, sizeof("\r\n") + 1, "\r\n");
        }
    p += std::snprintf(p, sizeof("\r\n") + 1, "\r\n");

    if (writetcpcli(ntrip->tcp, reinterpret_cast<unsigned char *>(buff), p - buff, msg) != p - buff)
        {
            return 0;
        }

    tracet(2, "reqntrip_c: send request state=%d ns=%" PRIdPTR "\n", ntrip->state, p - buff);
    tracet(5, "reqntrip_c: n=%" PRIdPTR " buff=\n%s\n", p - buff, buff);
    ntrip->state = 1;
    return 1;
}


/* test ntrip server response ------------------------------------------------*/
int rspntrip_s(ntrip_t *ntrip, char *msg)
{
    int i;
    int nb;
    char *p;
    char *q;

    tracet(3, "rspntrip_s: state=%d nb=%d\n", ntrip->state, ntrip->nb);
    ntrip->buff[ntrip->nb] = '0';
    tracet(5, "rspntrip_s: n=%d buff=\n%s\n", ntrip->nb, ntrip->buff);

    if ((p = strstr(reinterpret_cast<char *>(ntrip->buff), NTRIP_RSP_OK_SVR)))
        { /* ok */
            q = reinterpret_cast<char *>(ntrip->buff);
            p += strlen(NTRIP_RSP_OK_SVR);
            ntrip->nb -= p - q;
            for (i = 0; i < ntrip->nb; i++)
                {
                    *q++ = *p++;
                }
            ntrip->state = 2;
            std::snprintf(msg, MAXSTRMSG, "%s/%s", ntrip->tcp->svr.saddr, ntrip->mntpnt);
            tracet(2, "rspntrip_s: response ok nb=%d\n", ntrip->nb);
            return 1;
        }
    if ((p = strstr(reinterpret_cast<char *>(ntrip->buff), NTRIP_RSP_ERROR)))
        { /* error */
            nb = ntrip->nb < MAXSTATMSG ? ntrip->nb : MAXSTATMSG;
            // strncpy(msg, (char *)ntrip->buff, nb); This line triggers a warning. Replaced by;
            std::string s_aux(reinterpret_cast<char *>(ntrip->buff));
            s_aux.resize(nb, '\0');
            for (i = 0; i < nb; i++)
                {
                    msg[i] = s_aux[i];
                }

            msg[nb] = 0;
            tracet(1, "rspntrip_s: %s nb=%d\n", msg, ntrip->nb);
            ntrip->nb = 0;
            ntrip->buff[0] = '\0';
            ntrip->state = 0;
            discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
        }
    else if (ntrip->nb >= NTRIP_MAXRSP)
        { /* buffer overflow */
            std::snprintf(msg, MAXSTRMSG, "response overflow");
            tracet(1, "rspntrip_s: response overflow nb=%d\n", ntrip->nb);
            ntrip->nb = 0;
            ntrip->buff[0] = '\0';
            ntrip->state = 0;
            discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
        }
    tracet(5, "rspntrip_s: exit state=%d nb=%d\n", ntrip->state, ntrip->nb);
    return 0;
}


/* test ntrip client response ------------------------------------------------*/
int rspntrip_c(ntrip_t *ntrip, char *msg)
{
    int i;
    char *p;
    char *q;

    tracet(3, "rspntrip_c: state=%d nb=%d\n", ntrip->state, ntrip->nb);
    ntrip->buff[ntrip->nb] = '0';
    tracet(5, "rspntrip_c: n=%d buff=\n%s\n", ntrip->nb, ntrip->buff);

    if ((p = strstr(reinterpret_cast<char *>(ntrip->buff), NTRIP_RSP_OK_CLI)))
        { /* ok */
            q = reinterpret_cast<char *>(ntrip->buff);
            p += strlen(NTRIP_RSP_OK_CLI);
            ntrip->nb -= p - q;
            for (i = 0; i < ntrip->nb; i++)
                {
                    *q++ = *p++;
                }
            ntrip->state = 2;
            std::snprintf(msg, MAXSTRMSG, "%s/%s", ntrip->tcp->svr.saddr, ntrip->mntpnt);
            tracet(2, "rspntrip_c: response ok nb=%d\n", ntrip->nb);
            return 1;
        }
    if ((p = strstr(reinterpret_cast<char *>(ntrip->buff), NTRIP_RSP_SRCTBL)))
        { /* source table */
            if (!*ntrip->mntpnt)
                { /* source table request */
                    ntrip->state = 2;
                    std::snprintf(msg, MAXSTRMSG, "source table received");
                    tracet(2, "rspntrip_c: receive source table nb=%d\n", ntrip->nb);
                    return 1;
                }
            std::snprintf(msg, MAXSTRMSG, "no mountp. reconnect...");
            tracet(2, "rspntrip_c: no mount point nb=%d\n", ntrip->nb);
            ntrip->nb = 0;
            ntrip->buff[0] = '\0';
            ntrip->state = 0;
            discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
        }
    else if ((p = strstr(reinterpret_cast<char *>(ntrip->buff), NTRIP_RSP_HTTP)))
        { /* http response */
            if ((q = strchr(p, '\r')))
                {
                    *q = '\0';
                }
            else
                {
                    ntrip->buff[128] = '\0';
                }
            std::strncpy(msg, p, MAXSTRMSG);
            tracet(1, "rspntrip_s: %s nb=%d\n", msg, ntrip->nb);
            ntrip->nb = 0;
            ntrip->buff[0] = '\0';
            ntrip->state = 0;
            discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
        }
    else if (ntrip->nb >= NTRIP_MAXRSP)
        { /* buffer overflow */
            std::snprintf(msg, MAXSTRMSG, "response overflow");
            tracet(1, "rspntrip_s: response overflow nb=%d\n", ntrip->nb);
            ntrip->nb = 0;
            ntrip->buff[0] = '\0';
            ntrip->state = 0;
            discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
        }
    tracet(5, "rspntrip_c: exit state=%d nb=%d\n", ntrip->state, ntrip->nb);
    return 0;
}


/* wait ntrip request/response -----------------------------------------------*/
int waitntrip(ntrip_t *ntrip, char *msg)
{
    int n;
    char *p;

    tracet(4, "waitntrip: state=%d nb=%d\n", ntrip->state, ntrip->nb);

    if (ntrip->state < 0)
        {
            return 0; /* error */
        }

    if (ntrip->tcp->svr.state < 2)
        {
            ntrip->state = 0; /* tcp disconnected */
        }

    if (ntrip->state == 0)
        { /* send request */
            if (!(ntrip->type == 0 ? reqntrip_s(ntrip, msg) : reqntrip_c(ntrip, msg)))
                {
                    return 0;
                }
            tracet(2, "waitntrip: state=%d nb=%d\n", ntrip->state, ntrip->nb);
        }
    if (ntrip->state == 1)
        { /* read response */
            p = reinterpret_cast<char *>(ntrip->buff) + ntrip->nb;
            if ((n = readtcpcli(ntrip->tcp, reinterpret_cast<unsigned char *>(p), NTRIP_MAXRSP - ntrip->nb - 1, msg)) == 0)
                {
                    tracet(5, "waitntrip: readtcp n=%d\n", n);
                    return 0;
                }
            ntrip->nb += n;
            ntrip->buff[ntrip->nb] = '\0';

            /* wait response */
            return ntrip->type == 0 ? rspntrip_s(ntrip, msg) : rspntrip_c(ntrip, msg);
        }
    return 1;
}


/* open ntrip ----------------------------------------------------------------*/
ntrip_t *openntrip(const char *path, int type, char *msg)
{
    ntrip_t *ntrip;
    int i;
    char addr[256] = "";
    char port[256] = "";
    char tpath[MAXSTRPATH];

    tracet(3, "openntrip: path=%s type=%d\n", path, type);

    if (!(ntrip = static_cast<ntrip_t *>(malloc(sizeof(ntrip_t)))))
        {
            return nullptr;
        }

    ntrip->state = 0;
    ntrip->type = type; /* 0:server, 1:client */
    ntrip->nb = 0;
    ntrip->url[0] = '\0';
    ntrip->mntpnt[0] = ntrip->user[0] = ntrip->passwd[0] = ntrip->str[0] = '\0';
    for (i = 0; i < NTRIP_MAXRSP; i++)
        {
            ntrip->buff[i] = 0;
        }

    /* decode tcp/ntrip path */
    decodetcppath(path, addr, port, ntrip->user, ntrip->passwd, ntrip->mntpnt,
        ntrip->str);

    /* use default port if no port specified */
    if (!*port)
        {
            std::snprintf(port, sizeof(port), "%d", type ? NTRIP_CLI_PORT : NTRIP_SVR_PORT);
        }
    std::snprintf(tpath, MAXSTRPATH, "%s:%s", addr, port);

    /* ntrip access via proxy server */
    if (*proxyaddr)
        {
            std::string s_aux = "http://"s + std::string(tpath);
            int n = s_aux.length();
            if (n < 256)
                {
                    for (int k = 0; k < n; k++)
                        {
                            ntrip->url[k] = s_aux[k];
                        }
                }
            std::strncpy(tpath, proxyaddr, MAXSTRPATH);
        }
    /* open tcp client stream */
    if (!(ntrip->tcp = opentcpcli(tpath, msg)))
        {
            tracet(1, "openntrip: opentcp error\n");
            free(ntrip);
            return nullptr;
        }
    return ntrip;
}


/* close ntrip ---------------------------------------------------------------*/
void closentrip(ntrip_t *ntrip)
{
    tracet(3, "closentrip: state=%d\n", ntrip->state);
    closetcpcli(ntrip->tcp);
    free(ntrip);
}


/* read ntrip ----------------------------------------------------------------*/
int readntrip(ntrip_t *ntrip, unsigned char *buff, int n, char *msg)
{
    int nb;

    tracet(4, "readntrip: n=%d\n", n);

    if (!waitntrip(ntrip, msg))
        {
            return 0;
        }
    if (ntrip->nb > 0)
        { /* read response buffer first */
            nb = ntrip->nb <= n ? ntrip->nb : n;
            memcpy(buff, ntrip->buff + ntrip->nb - nb, nb);
            ntrip->nb = 0;
            return nb;
        }
    return readtcpcli(ntrip->tcp, buff, n, msg);
}


/* write ntrip ---------------------------------------------------------------*/
int writentrip(ntrip_t *ntrip, unsigned char *buff, int n, char *msg)
{
    tracet(3, "writentrip: n=%d\n", n);

    if (!waitntrip(ntrip, msg))
        {
            return 0;
        }
    return writetcpcli(ntrip->tcp, buff, n, msg);
}


/* get state ntrip -----------------------------------------------------------*/
int statentrip(ntrip_t *ntrip)
{
    return !ntrip ? 0 : (ntrip->state == 0 ? ntrip->tcp->svr.state : ntrip->state);
}


/* decode ftp path ----------------------------------------------------------*/
void decodeftppath(const char *path, char *addr, char *file, char *user,
    char *passwd, int *topts)
{
    char buff[MAXSTRPATH] = "";
    char *p;
    char *q;

    tracet(4, "decodeftpath: path=%s\n", path);

    if (user)
        {
            *user = '\0';
        }
    if (passwd)
        {
            *passwd = '\0';
        }
    if (topts)
        {
            topts[0] = 0;    /* time offset in path (s) */
            topts[1] = 3600; /* download interval (s) */
            topts[2] = 0;    /* download time offset (s) */
            topts[3] = 0;    /* retry interval (s) (0: no retry) */
        }
    if (strlen(path) < MAXSTRPATH)
        {
            std::strncpy(buff, path, MAXSTRPATH);
            buff[MAXSTRPATH - 1] = '\0';
        }

    if ((p = strchr(buff, '/')))
        {
            if ((q = strstr(p + 1, "::")))
                {
                    *q = '\0';
                    if (topts)
                        {
                            sscanf(q + 2, "T=%d, %d, %d, %d", topts, topts + 1, topts + 2, topts + 3);
                        }
                }
            std::strncpy(file, p + 1, 1024);
            file[1023] = '\0';
            *p = '\0';
        }
    else
        {
            file[0] = '\0';
        }

    if ((p = strrchr(buff, '@')))
        {
            *p++ = '\0';
            if ((q = strchr(buff, ':')))
                {
                    *q = '\0';
                    if (passwd)
                        {
                            std::strncpy(passwd, q + 1, 256);
                            passwd[255] = '\0';
                        }
                }
            if (user)
                {
                    std::memcpy(user, buff, 256);
                    user[255] = '\0';
                }
        }
    else
        {
            p = buff;
        }

    std::strncpy(addr, p, 1024);
    addr[1023] = '\0';
}


/* next download time --------------------------------------------------------*/
gtime_t nextdltime(const int *topts, int stat)
{
    gtime_t time;
    double tow;
    int week;
    int tint;

    tracet(3, "nextdltime: topts=%d %d %d %d stat=%d\n", topts[0], topts[1],
        topts[2], topts[3], stat);

    /* current time (gpst) */
    time = utc2gpst(timeget());
    tow = time2gpst(time, &week);

    /* next retry time */
    if (stat == 0 && topts[3] > 0)
        {
            tow = (floor((tow - topts[2]) / topts[3]) + 1.0) * topts[3] + topts[2];
            return gpst2time(week, tow);
        }
    /* next interval time */
    tint = topts[1] <= 0 ? 3600 : topts[1];
    tow = (floor((tow - topts[2]) / tint) + 1.0) * tint + topts[2];
    time = gpst2time(week, tow);

    return time;
}


/* ftp thread ----------------------------------------------------------------*/
void *ftpthread(void *arg)
{
    auto *ftp = static_cast<ftp_t *>(arg);

    tracet(3, "ftpthread:\n");

    if (!*localdir)
        {
            tracet(1, "no local directory\n");
            ftp->error = 11;
            ftp->state = 3;
            return nullptr;
        }
    /* replace keyword in file path and local path */
    auto time = timeadd(utc2gpst(timeget()), ftp->topts[0]);

    std::string remote;
    reppath(ftp->file, remote, time, "", "");
    auto remotePath = fs::path(remote);

    auto local = fs::path(localdir);
    local /= remotePath.filename();

    auto errfile = fs::path(local);
    errfile.replace_extension("err");

    /* if local file exist, skip download */
    auto tmpfile = local;
    for (auto ext : {".z", ".gz", ".zip", ".Z", ".GZ", ".ZIP"})  // NOLINT(readability-qualified-auto): auto decoration is less readable
        {
            if (tmpfile.extension() == ext)
                {
                    tmpfile.replace_extension("");
                    break;
                }
        }
    if (fs::exists(tmpfile))
        {
            std::snprintf(ftp->local, 1024, "%s", tmpfile.c_str());  // NOLINT(runtime/printf)
            tracet(3, "ftpthread: file exists %s\n", ftp->local);
            ftp->state = 2;
            return nullptr;
        }

    std::string env;

    /* proxy settings for wget (ref [2]) */
    auto proxyopt = std::string();
    if (*proxyaddr)
        {
            auto proto = "ftp"s;
            if (ftp->proto) proto = "http"s;  // NOLINT(readability-braces-around-statements): adding braces reduces readability
            env = "set "s + proto + "_proxy=http://"s + std::string(proxyaddr) + " % ";
            proxyopt = "--proxy=on ";
        }

    /* download command (ref [2]) */
    auto cmd_str = std::string();
    if (ftp->proto == 0)
        { /* ftp */
            auto opt_str = "--ftp-user="s + std::string(ftp->user) + " --ftp-password="s + std::string(ftp->passwd) +
                           " --glob=off --passive-ftp "s + proxyopt + "s-t 1 -T "s + std::to_string(FTP_TIMEOUT) +
                           R"( -O ")" + local.native() + R"(")"s;

            // TODO: this uses shell syntax; consider escaping paths
            cmd_str = env + std::string(FTP_CMD) + " "s + opt_str + " "s +
                      R"("ftp://)"s + std::string(ftp->addr) + "/"s + remotePath.native() + R"(" 2> ")"s + errfile.native() + "\"\n"s;
        }
    else
        { /* http */
            auto opt_str = proxyopt + " -t 1 -T "s + std::to_string(FTP_TIMEOUT) + " -O \""s + local.native() + "\""s;

            cmd_str = env + std::string(FTP_CMD) + " "s + opt_str + " "s +
                      R"("http://)"s + std::string(ftp->addr) + "/"s + remotePath.native() + R"(" 2> ")"s + errfile.native() + "\"\n";
        }
    /* execute download command */
    errorlib::error_code ec;  // prevent exceptions
    auto ret = execcmd(cmd_str.c_str());
    if ((ret != 0))
        {
            if (fs::remove(local, ec) == false)
                {
                    trace(1, "Error removing file %s", local.c_str());
                }
            tracet(1, "execcmd error: cmd=%s ret=%d\n", cmd_str.data(), ret);
            ftp->error = ret;
            ftp->state = 3;
            return nullptr;
        }
    if (fs::remove(errfile, ec) == false)
        {
            trace(1, "Error removing file %s", errfile.c_str());
        }

    /* uncompress downloaded file */
    for (auto ext : {".z", ".gz", ".zip", ".Z", ".GZ", ".ZIP"})  // NOLINT(readability-qualified-auto): auto decoration is less readable
        {
            if (local.extension() == ext)
                {
                    char tmpfile_arg[1024];
                    ret = rtk_uncompress(local.c_str(), tmpfile_arg);
                    if (ret != 0)  // success
                        {
                            if (fs::remove(local, ec) == false)
                                {
                                    trace(1, "Error removing file %s", local.c_str());
                                }
                            local = tmpfile_arg;
                        }
                    else
                        {
                            tracet(1, "file uncompact error: %s\n", local.c_str());
                            ftp->error = 12;
                            ftp->state = 3;
                            return nullptr;
                        }
                    break;
                }
        }
    int ret2 = std::snprintf(ftp->local, 1024, "%s", local.c_str());  // NOLINT(runtime/printf)
    if (ret2 < 0 || ret2 >= 1024)
        {
            tracet(3, "Error reading ftp local\n");
        }
    ftp->state = 2; /* ftp completed */

    tracet(3, "ftpthread: complete cmd=%s\n", cmd_str.data());
    return nullptr;
}


/* open ftp ------------------------------------------------------------------*/
ftp_t *openftp(const char *path, int type, char *msg)
{
    ftp_t *ftp;

    tracet(3, "openftp: path=%s type=%d\n", path, type);

    msg[0] = '\0';

    if (!(ftp = static_cast<ftp_t *>(malloc(sizeof(ftp_t)))))
        {
            return nullptr;
        }

    ftp->state = 0;
    ftp->proto = type;
    ftp->error = 0;
    ftp->thread = pthread_t();
    ftp->local[0] = '\0';

    /* decode ftp path */
    decodeftppath(path, ftp->addr, ftp->file, ftp->user, ftp->passwd, ftp->topts);

    /* set first download time */
    ftp->tnext = timeadd(timeget(), 10.0);

    return ftp;
}


/* close ftp -----------------------------------------------------------------*/
void closeftp(ftp_t *ftp)
{
    tracet(3, "closeftp: state=%d\n", ftp->state);

    if (ftp->state != 1)
        {
            free(ftp);
        }
}


/* read ftp ------------------------------------------------------------------*/
int readftp(ftp_t *ftp, unsigned char *buff, int n, char *msg)
{
    gtime_t time;
    unsigned char *p;
    unsigned char *q;

    tracet(4, "readftp: n=%d\n", n);

    time = utc2gpst(timeget());

    if (timediff(time, ftp->tnext) < 0.0)
        { /* until download time? */
            return 0;
        }
    if (ftp->state <= 0)
        { /* ftp/http not executed? */
            ftp->state = 1;
            if (std::snprintf(msg, sizeof(ftp->addr), "%s://%s", ftp->proto ? "http" : "ftp", ftp->addr) < 0)
                {
                    tracet(1, "readftp: ftp address truncation\n");
                }

            if (pthread_create(&ftp->thread, nullptr, ftpthread, ftp))
                {
                    tracet(1, "readftp: ftp thread create error\n");
                    ftp->state = 3;
                    std::strncpy(msg, "ftp thread error", 17);
                    return 0;
                }
        }
    if (ftp->state <= 1)
        {
            return 0; /* ftp/http on going? */
        }

    if (ftp->state == 3)
        { /* ftp error */
            if (std::snprintf(msg, sizeof(ftp->addr), "%s error (%d)", ftp->proto ? "http" : "ftp", ftp->error) < 0)
                {
                    tracet(1, "readftp: ftp address truncation\n");
                }

            /* set next retry time */
            ftp->tnext = nextdltime(ftp->topts, 0);
            ftp->state = 0;
            return 0;
        }
    /* return local file path if ftp completed */
    p = buff;
    q = reinterpret_cast<unsigned char *>(ftp->local);
    while (*q && static_cast<int>(p - buff) < n)
        {
            *p++ = *q++;
        }
    p += std::snprintf(reinterpret_cast<char *>(p), sizeof("\r\n") + 1, "\r\n");

    /* set next download time */
    ftp->tnext = nextdltime(ftp->topts, 1);
    ftp->state = 0;

    std::strncpy(msg, "", 1);

    return static_cast<int>(p - buff);
}


/* get state ftp -------------------------------------------------------------*/
int stateftp(ftp_t *ftp)
{
    return !ftp ? 0 : (ftp->state == 0 ? 2 : (ftp->state <= 2 ? 3 : -1));
}


/* initialize stream environment -----------------------------------------------
 * initialize stream environment
 * args   : none
 * return : none
 *-----------------------------------------------------------------------------*/
void strinitcom()
{
    tracet(3, "strinitcom:\n");
}


/* initialize stream -----------------------------------------------------------
 * initialize stream struct
 * args   : stream_t *stream IO  stream
 * return : none
 *-----------------------------------------------------------------------------*/
void strinit(stream_t *stream)
{
    tracet(3, "strinit:\n");

    stream->type = 0;
    stream->mode = 0;
    stream->state = 0;
    stream->inb = stream->inr = stream->outb = stream->outr = 0;
    stream->tick = stream->tact = stream->inbt = stream->outbt = 0;
    initlock(&stream->lock);
    stream->port = nullptr;
    stream->path[0] = '\0';
    stream->msg[0] = '\0';
}


/* open stream -----------------------------------------------------------------
 * open stream for read or write
 * args   : stream_t *stream IO  stream
 *          int type         I   stream type (STR_SERIAL, STR_FILE, STR_TCPSVR, ...)
 *          int mode         I   stream mode (STR_MODE_???)
 *          char *path       I   stream path (see below)
 * return : status (0:error, 1:ok)
 * notes  : see reference [1] for NTRIP
 *          STR_FTP/HTTP needs "wget" in command search paths
 *
 * stream path ([] options):
 *
 *   STR_SERIAL   port[:brate[:bsize[:parity[:stopb[:fctr]]]]]
 *                    port  = COM?? (windows), tty??? (linuex, omit /dev/)
 *                    brate = bit rate     (bps)
 *                    bsize = bit size     (7|8)
 *                    parity= parity       (n|o|e)
 *                    stopb = stop bits    (1|2)
 *                    fctr  = flow control (off|rts)
 *   STR_FILE     file_path[::T][::+start][::xseppd][::S=swap]
 *                    ::T   = enable time tag
 *                    start = replay start offset (s)
 *                    speed = replay speed factor
 *                    swap  = output swap interval (hr) (0: no swap)
 *   STR_TCPSVR   :port
 *   STR_TCPCLI   address:port
 *   STR_NTRIPSVR user[:passwd]@address[:port]/moutpoint[:string]
 *   STR_NTRIPCLI [user[:passwd]]@address[:port][/mountpoint]
 *   STR_FTP      [user[:passwd]]@address/file_path[::T=poff[, tint[, toff, tret]]]]
 *   STR_HTTP     address/file_path[::T=poff[, tint[, toff, tret]]]]
 *                    poff  = time offset for path extension (s)
 *                    tint  = download interval (s)
 *                    toff  = download time offset (s)
 *                    tret  = download retry interval (s) (0:no retry)
 *-----------------------------------------------------------------------------*/
int stropen(stream_t *stream, int type, int mode, const char *path)
{
    tracet(3, "stropen: type=%d mode=%d path=%s\n", type, mode, path);

    stream->type = type;
    stream->mode = mode;
    if (strlen(path) < MAXSTRPATH)
        {
            std::strncpy(stream->path, path, MAXSTRPATH);
            stream->path[MAXSTRPATH - 1] = '\0';
        }
    stream->inb = stream->inr = stream->outb = stream->outr = 0;
    stream->tick = tickget();
    stream->inbt = stream->outbt = 0;
    stream->msg[0] = '\0';
    stream->port = nullptr;
    switch (type)
        {
        case STR_SERIAL:
            stream->port = openserial(path, mode, stream->msg);
            break;
        case STR_FILE:
            stream->port = openfile(path, mode, stream->msg);
            break;
        case STR_TCPSVR:
            stream->port = opentcpsvr(path, stream->msg);
            break;
        case STR_TCPCLI:
            stream->port = opentcpcli(path, stream->msg);
            break;
        case STR_NTRIPSVR:
            stream->port = openntrip(path, 0, stream->msg);
            break;
        case STR_NTRIPCLI:
            stream->port = openntrip(path, 1, stream->msg);
            break;
        case STR_FTP:
            stream->port = openftp(path, 0, stream->msg);
            break;
        case STR_HTTP:
            stream->port = openftp(path, 1, stream->msg);
            break;
        default:
            stream->state = 0;
            return 1;
        }
    stream->state = !stream->port ? -1 : 1;
    return stream->port != nullptr;
}


/* close stream ----------------------------------------------------------------
 * close stream
 * args   : stream_t *stream IO  stream
 * return : none
 *-----------------------------------------------------------------------------*/
void strclose(stream_t *stream)
{
    tracet(3, "strclose: type=%d mode=%d\n", stream->type, stream->mode);

    if (stream->port)
        {
            switch (stream->type)
                {
                case STR_SERIAL:
                    closeserial(static_cast<serial_t *>(stream->port));
                    break;
                case STR_FILE:
                    closefile(static_cast<file_t *>(stream->port));
                    break;
                case STR_TCPSVR:
                    closetcpsvr(static_cast<tcpsvr_t *>(stream->port));
                    break;
                case STR_TCPCLI:
                    closetcpcli(static_cast<tcpcli_t *>(stream->port));
                    break;
                case STR_NTRIPSVR:
                    closentrip(static_cast<ntrip_t *>(stream->port));
                    break;
                case STR_NTRIPCLI:
                    closentrip(static_cast<ntrip_t *>(stream->port));
                    break;
                case STR_FTP:
                    closeftp(static_cast<ftp_t *>(stream->port));
                    break;
                case STR_HTTP:
                    closeftp(static_cast<ftp_t *>(stream->port));
                    break;
                }
        }
    else
        {
            trace(2, "no port to close stream: type=%d\n", stream->type);
        }
    stream->type = 0;
    stream->mode = 0;
    stream->state = 0;
    stream->inr = stream->outr = 0;
    stream->path[0] = '\0';
    stream->msg[0] = '\0';
    stream->port = nullptr;
}


/* sync streams ----------------------------------------------------------------
 * sync time for streams
 * args   : stream_t *stream1 IO stream 1
 *          stream_t *stream2 IO stream 2
 * return : none
 * notes  : for replay files with time tags
 *-----------------------------------------------------------------------------*/
void strsync(stream_t *stream1, stream_t *stream2)
{
    file_t *file1;
    file_t *file2;
    if (stream1->type != STR_FILE || stream2->type != STR_FILE)
        {
            return;
        }
    file1 = static_cast<file_t *>(stream1->port);
    file2 = static_cast<file_t *>(stream2->port);
    if (file1 && file2)
        {
            syncfile(file1, file2);
        }
}


/* lock/unlock stream ----------------------------------------------------------
 * lock/unlock stream
 * args   : stream_t *stream I  stream
 * return : none
 *-----------------------------------------------------------------------------*/
void strlock(stream_t *stream) { rtk_lock(&stream->lock); }

void strunlock(stream_t *stream) { rtk_unlock(&stream->lock); }


/* read stream -----------------------------------------------------------------
 * read data from stream (unblocked)
 * args   : stream_t *stream I  stream
 *          unsigned char *buff O data buffer
 *          int    n         I  maximum data length
 * return : read data length
 * notes  : if no data, return immediately with no data
 *-----------------------------------------------------------------------------*/
int strread(stream_t *stream, unsigned char *buff, int n)
{
    unsigned int tick;
    char *msg = stream->msg;
    int nr;

    tracet(4, "strread: n=%d\n", n);

    if (!(stream->mode & STR_MODE_R) || !stream->port)
        {
            return 0;
        }

    strlock(stream);

    switch (stream->type)
        {
        case STR_SERIAL:
            nr = readserial(static_cast<serial_t *>(stream->port), buff, n, msg);
            break;
        case STR_FILE:
            nr = readfile(static_cast<file_t *>(stream->port), buff, n, msg);
            break;
        case STR_TCPSVR:
            nr = readtcpsvr(static_cast<tcpsvr_t *>(stream->port), buff, n, msg);
            break;
        case STR_TCPCLI:
            nr = readtcpcli(static_cast<tcpcli_t *>(stream->port), buff, n, msg);
            break;
        case STR_NTRIPCLI:
            nr = readntrip(static_cast<ntrip_t *>(stream->port), buff, n, msg);
            break;
        case STR_FTP:
            nr = readftp(static_cast<ftp_t *>(stream->port), buff, n, msg);
            break;
        case STR_HTTP:
            nr = readftp(static_cast<ftp_t *>(stream->port), buff, n, msg);
            break;
        default:
            strunlock(stream);
            return 0;
        }
    stream->inb += nr;
    tick = tickget();
    if (nr > 0)
        {
            stream->tact = tick;
        }

    if (static_cast<int>(tick - stream->tick) >= tirate)
        {
            stream->inr = (stream->inb - stream->inbt) * 8000 / (tick - stream->tick);
            stream->tick = tick;
            stream->inbt = stream->inb;
        }
    strunlock(stream);
    return nr;
}


/* write stream ----------------------------------------------------------------
 * write data to stream (unblocked)
 * args   : stream_t *stream I   stream
 *          unsigned char *buff I data buffer
 *          int    n         I   data length
 * return : status (0:error, 1:ok)
 * notes  : write data to buffer and return immediately
 *-----------------------------------------------------------------------------*/
int strwrite(stream_t *stream, unsigned char *buff, int n)
{
    unsigned int tick;
    char *msg = stream->msg;
    int ns;

    tracet(3, "strwrite: n=%d\n", n);

    if (!(stream->mode & STR_MODE_W) || !stream->port)
        {
            return 0;
        }

    strlock(stream);

    switch (stream->type)
        {
        case STR_SERIAL:
            ns = writeserial(static_cast<serial_t *>(stream->port), buff, n, msg);
            break;
        case STR_FILE:
            ns = writefile(static_cast<file_t *>(stream->port), buff, n, msg);
            break;
        case STR_TCPSVR:
            ns = writetcpsvr(static_cast<tcpsvr_t *>(stream->port), buff, n, msg);
            break;
        case STR_TCPCLI:
            ns = writetcpcli(static_cast<tcpcli_t *>(stream->port), buff, n, msg);
            break;
        case STR_NTRIPCLI:
        case STR_NTRIPSVR:
            ns = writentrip(static_cast<ntrip_t *>(stream->port), buff, n, msg);
            break;
        case STR_FTP:
        case STR_HTTP:
        default:
            strunlock(stream);
            return 0;
        }
    stream->outb += ns;
    tick = tickget();
    if (ns > 0)
        {
            stream->tact = tick;
        }

    if (static_cast<int>(tick - stream->tick) > tirate)
        {
            stream->outr = (stream->outb - stream->outbt) * 8000 / (tick - stream->tick);
            stream->tick = tick;
            stream->outbt = stream->outb;
        }
    strunlock(stream);
    return ns;
}


/* get stream status -----------------------------------------------------------
 * get stream status
 * args   : stream_t *stream I   stream
 *          char   *msg      IO  status message (NULL: no output)
 * return : status (-1:error, 0:close, 1:wait, 2:connect, 3:active)
 *-----------------------------------------------------------------------------*/
int strstat(stream_t *stream, char *msg)
{
    int state;

    tracet(4, "strstat:\n");

    strlock(stream);
    if (msg)
        {
            // strncpy(msg, stream->msg, MAXSTRMSG - 1); This line triggers a warning. Replaced by:
            std::string aux_s(stream->msg);
            aux_s.resize(MAXSTRMSG - 1, '0');
            for (int i = 0; i < MAXSTRMSG - 1; i++)
                {
                    msg[i] = aux_s[i];
                }
            msg[MAXSTRMSG - 1] = '\0';
        }
    if (!stream->port)
        {
            strunlock(stream);
            return stream->state;
        }
    switch (stream->type)
        {
        case STR_SERIAL:
            state = stateserial(static_cast<serial_t *>(stream->port));
            break;
        case STR_FILE:
            state = statefile(static_cast<file_t *>(stream->port));
            break;
        case STR_TCPSVR:
            state = statetcpsvr(static_cast<tcpsvr_t *>(stream->port));
            break;
        case STR_TCPCLI:
            state = statetcpcli(static_cast<tcpcli_t *>(stream->port));
            break;
        case STR_NTRIPSVR:
        case STR_NTRIPCLI:
            state = statentrip(static_cast<ntrip_t *>(stream->port));
            break;
        case STR_FTP:
            state = stateftp(static_cast<ftp_t *>(stream->port));
            break;
        case STR_HTTP:
            state = stateftp(static_cast<ftp_t *>(stream->port));
            break;
        default:
            strunlock(stream);
            return 0;
        }
    if (state == 2 && static_cast<int>(tickget() - stream->tact) <= TINTACT)
        {
            state = 3;
        }
    strunlock(stream);
    return state;
}


/* get stream statistics summary -----------------------------------------------
 * get stream statistics summary
 * args   : stream_t *stream I   stream
 *          int    *inb      IO   bytes of input  (NULL: no output)
 *          int    *inr      IO   bps of input    (NULL: no output)
 *          int    *outb     IO   bytes of output (NULL: no output)
 *          int    *outr     IO   bps of output   (NULL: no output)
 * return : none
 *-----------------------------------------------------------------------------*/
void strsum(stream_t *stream, int *inb, int *inr, int *outb, int *outr)
{
    tracet(4, "strsum:\n");

    strlock(stream);
    if (inb)
        {
            *inb = stream->inb;
        }
    if (inr)
        {
            *inr = stream->inr;
        }
    if (outb)
        {
            *outb = stream->outb;
        }
    if (outr)
        {
            *outr = stream->outr;
        }
    strunlock(stream);
}


/* set global stream options ---------------------------------------------------
 * set global stream options
 * args   : int    *opt      I   options
 *              opt[0]= inactive timeout (ms) (0: no timeout)
 *              opt[1]= interval to reconnect (ms)
 *              opt[2]= averaging time of data rate (ms)
 *              opt[3]= receive/send buffer size (bytes);
 *              opt[4]= file swap margin (s)
 *              opt[5]= reserved
 *              opt[6]= reserved
 *              opt[7]= reserved
 * return : none
 *-----------------------------------------------------------------------------*/
void strsetopt(const int *opt)
{
    tracet(3, "strsetopt: opt=%d %d %d %d %d %d %d %d\n", opt[0], opt[1], opt[2],
        opt[3], opt[4], opt[5], opt[6], opt[7]);

    toinact = 0 < opt[0] && opt[0] < 1000 ? 1000 : opt[0]; /* >=1s */
    ticonnect = opt[1] < 1000 ? 1000 : opt[1];             /* >=1s */
    tirate = opt[2] < 100 ? 100 : opt[2];                  /* >=0.1s */
    buffsize = opt[3] < 4096 ? 4096 : opt[3];              /* >=4096byte */
    fswapmargin = opt[4] < 0 ? 0 : opt[4];
}


/* set timeout time ------------------------------------------------------------
 * set timeout time
 * args   : stream_t *stream I   stream (STR_TCPCLI, STR_NTRIPCLI, STR_NTRIPSVR)
 *          int     inactive_timeout  I   inactive timeout (ms) (0: no timeout)
 *          int     tirecon  I   reconnect interval (ms) (0: no reconnect)
 * return : none
 *-----------------------------------------------------------------------------*/
void strsettimeout(stream_t *stream, int inactive_timeout, int tirecon)
{
    tcpcli_t *tcpcli;

    tracet(3, "strsettimeout: toinact=%d tirecon=%d\n", inactive_timeout, tirecon);

    if (stream->type == STR_TCPCLI)
        {
            tcpcli = static_cast<tcpcli_t *>(stream->port);
        }
    else if (stream->type == STR_NTRIPCLI || stream->type == STR_NTRIPSVR)
        {
            tcpcli = (static_cast<ntrip_t *>(stream->port))->tcp;
        }
    else
        {
            return;
        }

    tcpcli->toinact = inactive_timeout;
    tcpcli->tirecon = tirecon;
}


/* set local directory ---------------------------------------------------------
 * set local directory path for ftp/http download
 * args   : char   *dir      I   directory for download files
 * return : none
 *-----------------------------------------------------------------------------*/
void strsetdir(const char *dir)
{
    tracet(3, "strsetdir: dir=%s\n", dir);
    if (strlen(dir) < 1024)
        {
            std::strncpy(localdir, dir, 1024);
            localdir[1023] = '\0';
        }
}


/* set http/ntrip proxy address ------------------------------------------------
 * set http/ntrip proxy address
 * args   : char   *addr     I   http/ntrip proxy address <address>:<port>
 * return : none
 *-----------------------------------------------------------------------------*/
void strsetproxy(const char *addr)
{
    tracet(3, "strsetproxy: addr=%s\n", addr);
    if (strlen(addr) < 256)
        {
            std::strncpy(proxyaddr, addr, 256);
            proxyaddr[255] = '\0';
        }
}


/* get stream time -------------------------------------------------------------
 * get stream time
 * args   : stream_t *stream I   stream
 * return : current time or replay time for playback file
 *-----------------------------------------------------------------------------*/
gtime_t strgettime(stream_t *stream)
{
    file_t *file;
    if (stream->type == STR_FILE && (stream->mode & STR_MODE_R) &&
        (file = static_cast<file_t *>(stream->port)))
        {
            return timeadd(file->time, file->start); /* replay start time */
        }
    return utc2gpst(timeget());
}


/* send nmea request -----------------------------------------------------------
 * send nmea gpgga message to stream
 * args   : stream_t *stream I   stream
 *          double *pos      I   position {x, y, z} (ecef) (m)
 * return : none
 *-----------------------------------------------------------------------------*/
void strsendnmea(stream_t *stream, const double *pos)
{
    sol_t sol = {{0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, '0', '0', '0', 0, 0, 0};
    unsigned char buff[1024];
    int i;
    int n;

    tracet(3, "strsendnmea: pos=%.3f %.3f %.3f\n", pos[0], pos[1], pos[2]);

    sol.stat = SOLQ_SINGLE;
    sol.time = utc2gpst(timeget());
    for (i = 0; i < 3; i++)
        {
            sol.rr[i] = pos[i];
        }
    n = outnmea_gga(buff, &sol);
    strwrite(stream, buff, n);
}


/* generate general hex message ----------------------------------------------*/
int gen_hex(const char *msg, unsigned char *buff)
{
    unsigned char *q = buff;
    char mbuff[1024] = "";
    char *args[256];
    char *p;
    unsigned int byte;
    int i;
    int narg = 0;

    trace(4, "gen_hex: msg=%s\n", msg);

    strncpy(mbuff, msg, 1023);
    mbuff[1022] = '\0';
    for (p = strtok(mbuff, " "); p && narg < 256; p = strtok(nullptr, " "))
        {
            args[narg++] = p;
        }
    for (i = 0; i < narg; i++)
        {
            if (sscanf(args[i], "%x", &byte))
                {
                    *q++ = static_cast<unsigned char>(byte);
                }
        }
    return static_cast<int>(q - buff);
}


/* send receiver command -------------------------------------------------------
 * send receiver commands to stream
 * args   : stream_t *stream I   stream
 *          char   *cmd      I   receiver command strings
 * return : none
 *-----------------------------------------------------------------------------*/
void strsendcmd(stream_t *str, const char *cmd)
{
    unsigned char buff[1024];
    const char *p = cmd;
    const char *q;
    char msg[1024];
    char cmdend[] = "\r\n";
    int n;
    int m;
    int ms;

    tracet(3, "strsendcmd: cmd=%s\n", cmd);

    for (;;)
        {
            for (q = p;; q++)
                {
                    if (*q == '\r' || *q == '\n' || *q == '\0')
                        {
                            break;
                        }
                }
            n = static_cast<int>(q - p);
            strncpy(msg, p, n);
            msg[n] = '\0';

            if (!*msg || *msg == '#')
                { /* null or comment */
                }
            else if (*msg == '!')
                { /* binary escape */
                    if (!strncmp(msg + 1, "WAIT", 4))
                        { /* wait */
                            if (sscanf(msg + 5, "%d", &ms) < 1)
                                {
                                    ms = 100;
                                }
                            if (ms > 3000)
                                {
                                    ms = 3000; /* max 3 s */
                                }
                            sleepms(ms);
                        }

                    // else if (!strncmp(msg+1, "UBX", 3))
                    // { /* ublox */
                    //     if ((m=gen_ubx(msg+4, buff))>0) strwrite(str, buff, m);
                    // }
                    // else if (!strncmp(msg+1, "STQ", 3))
                    // { /* skytraq */
                    //   if ((m=gen_stq(msg+4, buff))>0) strwrite(str, buff, m);
                    // }
                    // else if (!strncmp(msg+1, "NVS", 3))
                    // { /* nvs */
                    //   if ((m=gen_nvs(msg+4, buff))>0) strwrite(str, buff, m);
                    // }
                    // else if (!strncmp(msg+1, "LEXR", 4))
                    // { /* lex receiver */
                    //     if ((m=gen_lexr(msg+5, buff))>0) strwrite(str, buff, m);
                    // }
                    else if (!strncmp(msg + 1, "HEX", 3))
                        { /* general hex message */
                            if ((m = gen_hex(msg + 4, buff)) > 0)
                                {
                                    strwrite(str, buff, m);
                                }
                        }
                }
            else
                {
                    strwrite(str, reinterpret_cast<unsigned char *>(msg), n);
                    strwrite(str, reinterpret_cast<unsigned char *>(cmdend), 2);
                }
            if (*q == '\0')
                {
                    break;
                }

            p = q + 1;
        }
}
