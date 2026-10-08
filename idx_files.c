/*
  This file is part of MiST-firmware

  MiST-firmware is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  MiST-firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>
#include <string.h>

#include "idx_files.h"
#include "fat_compat.h"
#include "data_io.h"
#include "menu.h"
#include "osd.h"

// From menu.c — needed to reset cursor position when opening the IDX menu
extern unsigned char menusub;

#define IDX_EOT                 4
#define CHAR_IS_LINEEND(c)      (((c) == '\n') || ((c) == '\r'))
#define CHAR_IS_COMMENT(c)      (((c) == ';'))
#define CHAR_IS_QUOTE(c)        (((c) == '"'))
#define CHAR_IS_SPACE(c)        (((c) == ' ') || ((c) == '\t'))
#define IDX_LINE_SIZE           128
#define TAP_HEADER_SIZE         20
#define TAP_VERSION_OFFSET      0x0C
#define TAP_SIZE_OFFSET         0x10
#define MAX_IDX_ENTRIES_PREVIEW 20

static FIL idxfile;
static FIL tapfile;
static int idx_pt = 0;
static int lastidx = -1;
static char idxline[IDX_LINE_SIZE];
static int f_index;
static int empty_counter = 1;

// Read a single character from the IDX file using sector-buffered I/O
static char idx_getch(void)
{
    UINT br;

    if (idx_pt >= (int)f_size(&idxfile)) return 0;
    //else return sector_buffer[(idx_pt++)&0x1ff];

    if (!(idx_pt & 0x1ff)) {
        // reload buffer
        f_read(&idxfile, sector_buffer, 512, &br);
        // hexdump(sector_buffer, 512, 0);
    }

    return sector_buffer[(idx_pt++) & 0x1ff];
}

// Parse a single line from the IDX file.
// Returns the line text and its associated offset.
// Return value: 0 = normal line, IDX_EOT = end of file, 1 = parse error (unclosed quote)
static int idx_getline(char *line, int *offset)
{
    char c;
    char ignore = 0;
    char literal = 0;
    char leadingspace = 0;
    int i = 0;
    *offset = 0;

    while (1) {
        c = idx_getch();
        int is_end = (!c || CHAR_IS_LINEEND(c));
        int is_separator = (*offset == 0 && CHAR_IS_SPACE(c));

        if (is_separator || (is_end && *offset == 0 && i > 0)) {
            line[i] = 0;
            *offset = (int)strtoll(line, NULL, 0);
            i = 0;
            line[0] = '\0';
            if (*offset) leadingspace = 1;
            if (is_end) break;
            continue;
        }

        if (is_end) break;
        if (!CHAR_IS_SPACE(c) && *offset) leadingspace = 0;

        if (CHAR_IS_QUOTE(c) && !ignore) {
            literal ^= 1;
        } else if (CHAR_IS_COMMENT(c) && !ignore && !literal) {
            ignore++;
        } else if ((literal || !ignore) && i < (IDX_LINE_SIZE - 1) && !leadingspace) {
            line[i++] = c;
        }

        if (*offset == 0 && CHAR_IS_SPACE(c)) {
            line[i] = 0;
            *offset = (int)strtoll(line, NULL, 0);
            i = 0;
            line[0] = '\0';
            if (*offset) leadingspace = 1;
        }
    }
    line[i] = '\0';

    // Generate a placeholder name for entries that have an offset but no name
    if (*offset != 0) {
        int j;
        for (j = 0; line[j] && CHAR_IS_SPACE(line[j]); j++)
            ;
        if (line[j] == '\0') {
            snprintf(line, IDX_LINE_SIZE, "NO-NAME-%03d", empty_counter++);
        }
    }

    return (c == 0) ? IDX_EOT : (literal ? 1 : 0);
}

// Retrieve the IDX entry at the given index.
// Rewinds the file if seeking backwards. Returns pointer to static line buffer.
static char *idxitem(int idx, int *offset)
{
    if (idx <= lastidx) {
        idx_pt = 0;
        f_rewind(&idxfile);
        lastidx = -1;
        empty_counter = 1;
    }

    *offset = 0;
    idxline[0] = '\0';

    while (1) {
        int r = idx_getline(idxline, offset);
        if (idxline[0]) lastidx++;
        if (r == IDX_EOT || idx == lastidx) break;
    }

    return idxline;
}

// Write the TAP header with updated program size to the FPGA
static int send_tap_header(uint8_t *header_buf, FSIZE_t program_size,
                           uint8_t *tap_version_out)
{
    // Extract TAP version from header byte 0x0C
    *tap_version_out = header_buf[TAP_VERSION_OFFSET];

    // Update size field (bytes 0x10-0x13, little-endian)
    header_buf[TAP_SIZE_OFFSET + 0] = (program_size >>  0) & 0xFF;
    header_buf[TAP_SIZE_OFFSET + 1] = (program_size >>  8) & 0xFF;
    header_buf[TAP_SIZE_OFFSET + 2] = (program_size >> 16) & 0xFF;
    header_buf[TAP_SIZE_OFFSET + 3] = (program_size >> 24) & 0xFF;

    iprintf("IDX: TAP header - version=%d, size=%u\n",
            *tap_version_out, (unsigned int)program_size);

    EnableFpga();
    SPI(DIO_FILE_TX_DAT);
    // spi_write(header_buf, TAP_HEADER_SIZE);
    {
        int i;
        for (i = 0; i < TAP_HEADER_SIZE; i++)
            SPI(header_buf[i]);
    }
    DisableFpga();

    return 0;
}

// Transfer TAP program data from the file to the FPGA
static unsigned int send_tap_data(FSIZE_t program_size, unsigned int offset)
{
    UINT br;
    FRESULT res;
    FSIZE_t bytes_to_send = program_size;
    unsigned int chunk = 0, total_sent = 0;

    if (f_lseek(&tapfile, offset) != FR_OK) {
        iprintf("IDX: ERROR - seek to offset 0x%08X failed\n", offset);
        return 0;
    }

    while (bytes_to_send > 0) {
        UINT bytes_to_read = (bytes_to_send > SECTOR_BUFFER_SIZE) ?
                             SECTOR_BUFFER_SIZE : (UINT)bytes_to_send;

        DISKLED_ON
        res = f_read(&tapfile, sector_buffer, bytes_to_read, &br);
        DISKLED_OFF

        if (res != FR_OK || br == 0) {
            iprintf("IDX: ERROR - read failed at chunk %d (res=%d, br=%d)\n",
                    chunk, res, br);
            break;
        }

        EnableFpga();
        SPI(DIO_FILE_TX_DAT);
	// spi_write(sector_buffer, br);
        {
            unsigned int c;
            char *p;
            for (p = sector_buffer, c = 0; c < br; c++)
                SPI(*p++);
        }
        DisableFpga();

        total_sent += br;
        bytes_to_send -= br;
        chunk++;
    }

    iprintf("IDX: Transfer complete - %u bytes in %d chunks\n", total_sent, chunk);
    return total_sent;
}

static char idx_getmenupage(uint8_t idx, char action, menu_page_t *page)
{
    if (action == MENU_PAGE_EXIT) {
        f_close(&idxfile);
        f_close(&tapfile);
        return 0;
    }

    page->title = "IDX";
    page->flags = 0;
    page->timer = 0;
    page->stdexit = MENU_STD_EXIT;
    return 0;
}

static char idx_getmenuitem(uint8_t idx, char action, menu_item_t *item)
{
    int offset, next_offset;
    char *str;

    // Initialize all item fields to safe defaults on every call.
    // This is critical — the menu framework reads these fields after every
    // MENU_ACT_GET call. Uninitialized values (especially page, newsub,
    // newpage) cause unpredictable cursor positioning and menu behavior.
    item->item = "";
    item->active = 0;
    item->stipple = 0;
    item->page = 0;
    item->newpage = 0;
    item->newsub = 0;

    if (action == MENU_ACT_GET) {
        str = idxitem(idx, &offset);
        item->item = str;
        item->active = (str[0] != 0);
        return (str[0] != 0);

    } else if (action == MENU_ACT_SEL) {
        // Save the selected entry name before it gets overwritten
        char current_name[IDX_LINE_SIZE];
        str = idxitem(idx, &offset);
        strncpy(current_name, str, IDX_LINE_SIZE - 1);
        current_name[IDX_LINE_SIZE - 1] = '\0';

        FSIZE_t total_size = f_size(&tapfile);

        // Validate offset against TAP file size
        if ((FSIZE_t)offset >= total_size) {
            iprintf("IDX: ERROR - offset 0x%08X beyond TAP size 0x%08X\n",
                    offset, (unsigned int)total_size);
            f_close(&tapfile);
            CloseMenu();
            return 1;
        }

        // Get the next entry's offset to determine program boundaries
        idxitem(idx + 1, &next_offset);

        iprintf("IDX: load \"%s\" at 0x%08X\n", current_name, offset);
        f_close(&idxfile);

        // Read the TAP header (first 20 bytes of the TAP file)
        UINT br;
        FRESULT res;

        // Rewind to beginning to read the header
        f_lseek(&tapfile, 0);
        DISKLED_ON
        res = f_read(&tapfile, sector_buffer, TAP_HEADER_SIZE, &br);
        DISKLED_OFF

        if (res != FR_OK || br != TAP_HEADER_SIZE) {
            iprintf("IDX: ERROR - header read failed (res=%d, br=%d)\n", res, br);
            f_close(&tapfile);
            CloseMenu();
            return 1;
        }

        // Calculate program size from offset boundaries
        FSIZE_t end_pos = (next_offset > offset) ? (FSIZE_t)next_offset : total_size;
        FSIZE_t program_size = end_pos - (FSIZE_t)offset;

        iprintf("IDX: range [0x%08X - 0x%08X], size=%u\n",
                offset, (unsigned int)end_pos, (unsigned int)program_size);

        // Prepare FPGA transfer
        uint8_t tap_version;
        data_io_file_tx_prepare(&tapfile, f_index, "TAP");
        send_tap_header(sector_buffer, program_size, &tap_version);

        // Transfer program data
        unsigned int total_sent = send_tap_data(program_size, offset);

        data_io_file_tx_done();
        f_close(&tapfile);
        CloseMenu();
        return 1;
    }

    return 0;
}

// Open an IDX file, locate the corresponding TAP file, and set up the menu
static void handleidx(FIL *file, int index, const char *name, const char *ext)
{
    iprintf("IDX: open IDX %s\n", name);

    empty_counter = 1;
    f_rewind(file);
    idxfile = *file;
    idx_pt = 0;
    lastidx = -1;
    f_index = index;

    // Find the file extension position
    const char *fileExt = NULL;
    int len = strlen(name);
    while (len > 2) {
        if (name[len - 2] == '.') {
            fileExt = &name[len - 1];
            break;
        }
        len--;
    }

    if (!fileExt) {
        iprintf("IDX: ERROR - no file extension found\n");
        f_close(&idxfile);
        CloseMenu();
        return;
    }

    // Build the corresponding TAP filename (replace extension)
    // Use a fixed buffer instead of VLA for embedded safety
    char tap_name[FF_LFN_BUF + 1];
    if (len - 1 + 4 > (int)sizeof(tap_name)) {
        iprintf("IDX: ERROR - filename too long\n");
        f_close(&idxfile);
        CloseMenu();
        return;
    }
    memcpy(tap_name, name, len - 1);
    strcpy(&tap_name[len - 1], "TAP");

    iprintf("IDX: corresponding TAP: %s\n", tap_name);

    if (f_open(&tapfile, tap_name, FA_READ) != FR_OK) {
        iprintf("IDX: ERROR - cannot open TAP file\n");
        f_close(&idxfile);
        ErrorMessage("Unable to open the\ncorresponding TAP file!", 0);
        return;
    }

    iprintf("IDX: TAP opened, size=%u bytes\n", (unsigned int)f_size(&tapfile));

    // Preview IDX entries for debug output
    int temp_offset, entry = 0;
    idx_pt = 0;
    f_rewind(&idxfile);
    lastidx = -1;
    empty_counter = 1;

    iprintf("IDX entries:\n");
    while (entry < MAX_IDX_ENTRIES_PREVIEW) {
        char *e = idxitem(entry, &temp_offset);
        if (e[0] == 0) break;
        iprintf("  %2d: 0x%08X  \"%s\"\n", entry, temp_offset, e);
        entry++;
    }

    // Reset parser state for actual menu use
    idx_pt = 0;
    f_rewind(&idxfile);
    lastidx = -1;
    empty_counter = 1;

    // Reset cursor position so it starts at the first IDX entry.
    // SetupMenu does not reset menusub, so it retains the value from
    // the file browser or the 8-bit menu. On a 16-line OSD (firstline=2),
    // if menusub >= 3, the cursor lands on the second entry instead of the first.
    menusub = 0;
    SetupMenu(&idx_getmenupage, &idx_getmenuitem, NULL);
}

static data_io_processor_t idx_file = {"IDX", &handleidx};

void idx_files_init(void)
{
    data_io_add_processor(&idx_file);
}
