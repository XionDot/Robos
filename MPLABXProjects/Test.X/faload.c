/*****************************************
 * Formula AllCode Robot Buggy bootloader
 * 
 * Provides a loader option for non-
 * Windows users.
 * 
 * Laurence Tyler
 * Aberystywth University
 * 
 * 2018-03-14   LGT     Initial version 1.0
 * 
 *****************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <getopt.h>

#include <hidapi/hidapi.h>

#define VERSION "1.0"

#define SOH 1
#define EOT 4
#define DLE 16

#define BOOT_VERS   1 // Get bootloader version
#define BOOT_ERASE  2 // Erase flash memory
#define BOOT_PROG   3 // Program flash memory
#define BOOT_CRC    4 // Get program CRC
#define BOOT_EXEC   5 // Execute user program


#define ROBOT_VID 0x12bf
#define ROBOT_PID 0x00a1

#define MAXSTR 256

const char *short_opts = "enxcdh";

struct option long_opts[] = {
    {"erase", no_argument, NULL, 'e'},
    {"no-exec", no_argument, NULL, 'n'},
    {"exec", no_argument, NULL, 'x'},
    {"check", no_argument, NULL, 'c'},
    {"dump", no_argument, NULL, 'd'},
    {"help", no_argument, NULL, 'h'},
    {NULL, 0, NULL, 0}
};

const char *help_text = "Usage: faload [ OPTION... ] [ FILE ]\n"
"\n"
"   Program loader V" VERSION " for Formula AllCode robot buggies\n"
"\n"
"   faload [ -n ] FILE    Erase, load program from hex file & execute\n"
"       -n, --no-exec     Erase, load program but do not execute\n"
"   faload -x | --exec    Execute already-loaded program\n"
"   faload -e | --erase   Erase only\n"
"\n"
"   OPTIONS:\n"
"       -c, --check       Check response frames from robot\n"
"       -d, --dump        Dump frames in hexadecimal, for debugging\n"
"\n"
"LGT 2018-03-14\n"
;

#define USAGE "Usage: faload [ -c | -d ] [ -e | -x | -h | [ -n ] hexfile ]\n"

const char *hexdigit = "0123456789ABCDEF";
char st[1024];

int opt_dump = 0;   // Dump packet contents
int opt_erase = 1;  // Erase program memory
int opt_load = 1;   // Load program
int opt_exec = 1;   // Execute program
int opt_check = 0;  // Read (check) response packets

char *hf_name = NULL;

char *st_hid_error (hid_device *dev)
{
    int len;
    
    len = wcstombs(st, hid_error(dev), 1023);
    if (len == -1) {
        strcpy(st, "<ERROR!?>");
    }
    return st;
}

uint16_t crc_table[] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
};


uint16_t crc16 (uint8_t *data, int datalen)
{
    uint16_t crc, n;
    int i;
    
    crc = 0;
    for (i = 0; i < datalen; i++) {
        n = (crc >> 12) ^ (data[i] >> 4);
        crc = crc_table[n & 0x0f] ^ (crc << 4);
        n = (crc >> 12) ^ data[i];
        crc = crc_table[n & 0x0f] ^ (crc << 4);
    }
    
    return crc;
}

void put(uint8_t **seq, uint8_t val)
{
    uint8_t *p = *seq;
    if (val == SOH || val == EOT || val == DLE)
        *p++ = DLE;
    *p++ = val;
    *seq = p;
}

int mkpacket (uint8_t *pkt, uint8_t *data, int datalen)
{
    int i;
    uint8_t *p;
    uint16_t c;
    uint8_t b;
    
    p = pkt;
    *p++ = 0;   // HID report ID
    *p++ = SOH; // Start of data frame
    
    for (i = 0; i < datalen; i++) {
        put(&p, data[i]);
    }
    
    c = crc16(data, datalen);
    b = c & 0xff;
    put(&p, b);     // Checksum low
    b = (c >> 8) & 0xff;
    put(&p, b);     // Checksum high
    
    *p++ = EOT; // End of data frame
    
    return (p - pkt);
}

/* Trim trailing whitespace, inc. line terminators(s) */
/* NB: Modifies string in-place */

char *trim (char *s)
{
    int i;

    i = strlen(s);
    if (i == 0)
        return s;
    i--;    // Point at last actual char
    while (i >= 0 && isspace(s[i])) {
        s[i] = '\0';    // Overwrite whitespace
        i--;
    }
    return s;
}

int find_eot (uint8_t *pkt, int maxlen)
{
    int i = 0;
    
    while (i < maxlen) {
        if (pkt[i] == DLE)
            i++;
        if (pkt[i] == EOT)
            break;
        i++;
    }
    return i;
}

int decode_hex (char *buf, uint8_t *rec)
{
    int buflen, reclen;
    uint8_t b;
    char *p;
    
    reclen = 0;
    
    buflen = strlen(buf);
    if (buflen < 6) {
        fprintf(stderr, "** Hex file record too short: %s\n", buf);
        return 0;
    }
    if (buf[0] != ':') {
        fprintf(stderr, "** Hex file record doesn't start with colon: %s\n", buf);
        return 0;
    }
    
    rec[reclen++] = BOOT_PROG;    // Set up "program flash" command
    
    p = &buf[1];    // Skip initial colon
    while (*p && ((p - buf) < buflen)) {
        /* Upper nybble */
        if (!isxdigit(*p)) {
            fprintf(stderr, "** Bad hex digit in hex file record: %s\n", buf);
            return 0;
        }
        if (*p < 'A') {
            b = (*p & 0x0f) << 4;
        }
        else {
            b = ((*p & 0x0f) + 9) << 4;
        }
        p++;
        
        /* Lower nybble */
        if (!isxdigit(*p)) {
            fprintf(stderr, "** Bad hex digit in hex file record: %s\n", buf);
            return 0;
        }
        if (*p < 'A') {
            b = b | (*p & 0x0f);
        }
        else {
            b = b | ((*p & 0x0f) + 9);
        }
        p++;
        
        /* Save byte to binary record */
        rec[reclen++] = b;
    }
    return reclen;
}

char *hexdump (uint8_t *data, int datalen)
{
    char *s = st;
    int i;
    uint8_t b;
    
    *s = '\0';
    for (i = 0; i < datalen; i++) {
        b = data[i];
        *s++ = hexdigit[(b & 0xf0) >> 4];
        *s++ = hexdigit[b & 0x0f];
        *s++ = ' ';
        *s = '\0';
    }
    *(--s) = '\0';  // Overwrite final space
    return st;
}

int check_resp (hid_device *dev, uint8_t *pkt)
{
    int pktlen;
    
    if (hid_read(dev, pkt, 64) == -1) {
        fprintf(stderr, "** Error reading response from robot: %s\n", st_hid_error(dev));
        return -1;
    }
    pktlen = find_eot(pkt, 64);
    if (pktlen >= 64) {
        if (opt_dump) {
            fprintf(stdout, "PKT < %s\n", hexdump(pkt, 64));
        }
        fprintf(stderr, "** Bad response from robot - no EOT: %s...\n",   hexdump(pkt, 16));
        return -2;
    }
    if (opt_dump) {
        fprintf(stdout, "PKT < %s\n", hexdump(pkt, pktlen+1));
    }
    if (pkt[0] != SOH) {
        fprintf(stderr, "** Bad response from robot - no SOH: %s...\n",   hexdump(pkt, 16));
        return -3;
    }
    return pktlen+1;
}

void crack_params (int argc, char *argv[]) {
    
    int opt;
    int lindex;
    int e_flag = 0, n_flag = 0, x_flag = 0;
    int err_flag = 0, help_flag = 0;
    
    while ((opt = getopt_long(argc, argv, short_opts, long_opts, &lindex)) != -1) {
        
        switch (opt) {
            
            case 'e':   // Erase only
                if (n_flag||x_flag) {
                    err_flag++;
                    break;
                }
                e_flag++;
                opt_erase = 1;
                opt_load = 0;
                opt_exec = 0;
                break;
                
            case 'n':   // Load but don't execute
                if (e_flag||x_flag) {
                    err_flag++;
                    break;
                }
                n_flag++;
                opt_erase = 1;
                opt_load = 1;
                opt_exec = 0;
                break;
                
            case 'x':   // Execute only
                if (e_flag||n_flag) {
                    err_flag++;
                    break;
                }
                x_flag++;
                opt_erase = 0;
                opt_load = 0;
                opt_exec = 1;
                break;
                
            case 'c':   // Check for response packets
                opt_check = 1;
                break;
                
            case 'd':   // Dump packets for debugging
                opt_dump = 1;
                break;
                
            case 'h':   // Print help message & exit
                help_flag++;
                break;
                
            case '?':   // Unknown option
                fprintf(stderr, "%s\n", USAGE);
                exit(1);
        }
        
        if (help_flag) {
            fprintf(stdout, help_text);
            exit(0);
        }
        
        if (err_flag) {
            fprintf(stderr, "%s\n", USAGE);
            exit(1);
        }
    }
    /* Process non-option parameters */
    if ((argc - optind) == 0) {
        /* No file parameter */
        if (!(e_flag||x_flag)) {
            fprintf(stderr, "%s\n", USAGE);
            exit(1);
        }
    }
    else if ((argc - optind) == 1) {
        /* Exactly one file parameter */
        if (e_flag || x_flag) {
            fprintf(stderr, "%s\n", USAGE);
            exit(1);
        }
        hf_name = argv[optind];
    }
    else {
        /* Two or more file names */
        fprintf(stderr, "%s\n", USAGE);
            exit(1);
    }
}

int main (int argc, char *argv[])
{
    hid_device *dev = NULL;
    int res;
    int nrec;
    FILE *hf;
    char buf[MAXSTR];
    uint8_t rec[MAXSTR];
    uint8_t pkt[MAXSTR];
    
    int reclen, pktlen; 
    
    /* Process parameters */
    crack_params(argc, argv);
    /* Debugging */
    /*
    fprintf(stderr, "---> opt_dump:  %d\n", opt_dump);
    fprintf(stderr, "---> opt_erase: %d\n", opt_erase);
    fprintf(stderr, "---> opt_load:  %d\n", opt_load);
    fprintf(stderr, "---> opt_exec:  %d\n", opt_exec);
    fprintf(stderr, "---> opt_check: %d\n", opt_check);
    if (hf_name)
        fprintf(stderr, "---> hexfile: %s\n", hf_name);
    else
        fprintf(stderr, "---> hexfile: %s\n", "<NONE>");
    exit(0);
    */
    
    /* Open hex file for input */
    if (hf_name != NULL) {
        hf = fopen(hf_name, "r");
        if (hf == NULL) {
            fprintf(stderr, "** Hex file not found: %s\n", hf_name);
            exit(1);
        }
    }
    
    /* Initialise HID library */
    res = hid_init();
    if (res) {
        fprintf(stderr, "** Error opening HID API library\n");
        exit(1);
    }
    
    /* Open first robot found (if any) */
    dev = hid_open(ROBOT_VID, ROBOT_PID, NULL);
    if (dev == NULL) {
        fprintf(stderr, "** No robot device found\n   Press reset on robot to enter bootloader\n");
        exit(1);
    }
    
    if (opt_erase) {
        /* Erase flash memory */
        fprintf(stdout, "--- Sending ERASE command...\n");
        rec[0] = BOOT_ERASE;
        reclen = 1;
        pktlen = mkpacket(pkt, rec, reclen);
        /* DEBUG: Dump the packet */
        if (opt_dump) {
            fprintf(stdout, "PKT > %s\n", hexdump(pkt, pktlen));
        }
        if (hid_write(dev, pkt, pktlen) == -1) {
                fprintf(stderr, "** Error writing to robot: %s\n", st_hid_error(dev));
                goto bail_out;
        }
        /* Check response */
        pktlen = check_resp(dev, pkt);
        if (pktlen < 0)
            goto bail_out;
    }

    if (opt_load) {
        /* Read and process records */
        fprintf(stdout, "--- Sending PROGRAM records...\n");
        nrec = 0;
        while (fgets(buf, MAXSTR, hf) != NULL) {
            trim(buf);
            reclen = decode_hex(buf, rec);
            if (reclen == 0) {
                goto bail_out;
            }
            /* Make a program packet */
            pktlen = mkpacket(pkt, rec, reclen);
            if (opt_dump) {
                fprintf(stdout, "PKT > %s\n", hexdump(pkt, pktlen));
            }
            /* Send the packet as a HID report */
            if (hid_write(dev, pkt, pktlen) == -1) {
                fprintf(stderr, "** Error writing to robot: %s\n", st_hid_error(dev));
                goto bail_out;
            }
            nrec++;
            if (opt_check) {
                pktlen = check_resp(dev, pkt);
                if (pktlen < 0)
                    goto bail_out;
            }
        }
        fprintf(stdout, "    %d records sent.\n", nrec);
    }
    
    if (opt_exec) {
        fprintf(stdout, "--- Sending EXECUTE command...\n");
        rec[0] = BOOT_EXEC;
        reclen = 1;
        pktlen = mkpacket(pkt, rec, reclen);
        if (opt_dump) {
            fprintf(stdout, "PKT > %s\n", hexdump(pkt, pktlen));
        }
        if (hid_write(dev, pkt, pktlen) == -1) {
                fprintf(stderr, "** Error writing to robot: %s\n", st_hid_error(dev));
                goto bail_out;
        }
    }

    /* No return from that, so close down */
    fclose(hf);
    hid_close(dev);
    hid_exit();
    return 0;
    
    /* Error exit */
    
bail_out:
    fclose(hf);
    hid_close(dev);
    hid_exit();
    exit(1);

}    

