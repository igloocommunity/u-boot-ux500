/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Spjalle Joelson for ST-Ericsson
 * Licensed under GPLv2.
 *
 * Cleanups and comments: Michael Brandt <michael.brandt@stericsson.com>
 */

/* cmd_cdump.c - crash dump commands, require FAT write support */

#include <common.h>
#include <command.h>
#include "malloc.h"
#include <mmc.h>
#include <asm/io.h>

#include <asm/setup.h>
#include <elf.h>
#include <fat.h>
#include <asm/arch/hardware.h>

#define TWD_WDOG_LOAD			0x20

/*
 * Note: with the Rockbox FAT support, the file path must be an absolute path,
 * i.e. with leading /.
 */
static char *crash_filename = "/cdump.elf";

static void kick_mpcore_wdt(void)
{
	unsigned long mpcore_wdt_loadreg;

	mpcore_wdt_loadreg = readl(U8500_TWD_BASE + TWD_WDOG_LOAD);
	/*
	 * According to the linux mpcore_wdt driver a different value needs to
	 * be written to the load register every time
	 */
	mpcore_wdt_loadreg = mpcore_wdt_loadreg ^ 0x1;
	writel(mpcore_wdt_loadreg, U8500_TWD_BASE + TWD_WDOG_LOAD);
}

/*
 * Check ELF header
 */
static int check_elfhdr(Elf32_Ehdr *elfhdr_addr)
{
	unsigned char *elfhdr = (unsigned char *) elfhdr_addr;

	/* check ELF core image header MAGIC */
	if (memcmp(elfhdr, ELFMAG, SELFMAG) != 0)
		return 1;

	/* check that this is ELF32 */
	if (elfhdr[EI_CLASS] == ELFCLASS32)
		return 0;

	return 1;
}

/*
 * Write a chunk
 */
static int write_chunk(int fd, void *addr, size_t count)
{
	size_t bytes;

	bytes = write(fd, addr, count);
	if (bytes != count) {
		printf("write error\n");
		close(fd);
		return -1;
	}
	return 0;
}

/*
 * Write a big chunk with 'progress' indicator '.' for every MiB
 */
static int write_big_chunk(int fd, void *addr, size_t count)
{
	unsigned char *a = addr;
	size_t total = 0;

	if (count < 0x100000)
		return write_chunk(fd, addr, count);
	/* if large chunk then print dot to show progress */
	while (total < count) {
		size_t bytes = count - total;

		kick_mpcore_wdt();

		if (bytes > 0x100000)
			bytes = 0x100000;
		if (write_chunk(fd, a, bytes))
			return -1;
		putc('.');
		total += bytes;
		a += bytes;
	}
	putc('\n');
	return 0;
}

/*
 * Open the dump file for writing. Create if it not exists.
 * Note that with the Rockbox FAT support, the file path must be an absolute
 * path, i.e. with leading /.
 */
static int open_create(const char *filename)
{
	int fd;

	fd = open(filename, O_WRONLY | O_CREAT);
	if (fd < 0)
		printf("%s open error\n", filename);
	return fd;
}

/*
 * Check program header and segment
 * Truncate note segments.
 * Return segment size.
 */
static u32 check_phdr(Elf32_Phdr *proghdr)
{
	u32 i;
	u32 *note;
	Elf32_Phdr *phdr = proghdr;

	if (phdr->p_type == PT_NOTE) {
		/* see Linux kernel/kexec.c:append_elf_note() */
		note = (u32 *)(phdr->p_paddr);
		for (i = 0; i < phdr->p_filesz/4;) {
			if (note[i] == 0 && note[i+1] == 0 && note[i+2] == 0)
				return i*4;
			i += 3 + (note[i] + 3) / 4 + (note[i+1] + 3) / 4;
		}
	}

	return phdr->p_filesz;
}

/*
 * Dump crash to file
 */
static int write_elf(Elf32_Ehdr *elfhdr_addr, int fd)
{
	Elf32_Ehdr *oldhdr = elfhdr_addr;
	Elf32_Ehdr *ehdr;
	Elf32_Phdr *phdr;
	u32 i;
	u32 offset;
	u32 tot;
	u32 phdr_cnt;
	u32 notes_cnt = 0;
	u32 save;
	u32 len;

	offset = oldhdr->e_ehsize + oldhdr->e_phentsize * oldhdr->e_phnum;
	ehdr = (Elf32_Ehdr *) malloc(offset);
	if (ehdr == NULL) {
		debug("elf header alloc error\n");
		return -1;
	}
	memcpy(ehdr, oldhdr, offset);

	/*
	 * check program header entries and update length
	 * for merged PT_NOTE segments
	 */
	tot = 0;
	phdr_cnt = ehdr->e_phnum;
	debug("phdr_cnt=%d\n", phdr_cnt);
	for (i = 0; i < phdr_cnt; i++) {
		phdr = (Elf32_Phdr *) ((char *) ehdr + ehdr->e_ehsize +
				       i * ehdr->e_phentsize);
		len = check_phdr(phdr);
		debug("prog hdr %d: %x ad %x len %x adjusted to %x\n",
		      i, (u32) phdr, phdr->p_paddr, phdr->p_filesz, len);
		phdr->p_filesz = len;
		phdr->p_memsz = len;
		if (phdr->p_type == PT_NOTE) {	/* note segment */
			tot += len;
			notes_cnt++;
		}
	}
	debug("Length of %d note segments: %x\n", notes_cnt, tot);

	/*
	 * all PT_NOTE segments have been merged into one.
	 * Update ELF Header accordingly
	 */
	ehdr->e_phnum = phdr_cnt - notes_cnt + 1;

	/* write elf header into file on sdcard */
	if (write_chunk(fd, ehdr, (size_t) ehdr->e_ehsize)) {
		free(ehdr);
		return -1;
	}

	/* write program headers into file on sdcard */
	offset = ehdr->e_ehsize + ehdr->e_phentsize * ehdr->e_phnum;
	debug("Write Phdr: proghdr_cnt=%d\n", phdr_cnt);
	for (i = 0; i < phdr_cnt; i++) {
		phdr = (Elf32_Phdr *) ((char *)ehdr + ehdr->e_ehsize +
				       i * ehdr->e_phentsize);
		save = phdr->p_filesz;
		if (i == 0) {
			phdr->p_filesz = tot;
			phdr->p_memsz = tot;
		} else if (phdr->p_type == PT_NOTE) /* note segment */
			continue;
		phdr->p_offset = offset;
		debug("prog hdr %d: %x ad %x len %x off %x\n",
		       i, (u32) phdr, phdr->p_paddr, phdr->p_filesz,
		       phdr->p_offset);
		offset += phdr->p_filesz;
		if (write_chunk(fd, (void *) phdr, (size_t)
				ehdr->e_phentsize)) {
			free(ehdr);
			return -1;
		}
		phdr->p_filesz = save;
		phdr->p_memsz = save;
	}

	/* write segments into file on sdcard */
	debug("write segments...\n");
	for (i = 0; i < phdr_cnt; i++) {
		phdr = (Elf32_Phdr *) ((char *) ehdr + ehdr->e_ehsize +
				       i * ehdr->e_phentsize);
		if (phdr->p_type > PT_NULL) {
			if (write_big_chunk(fd, (void *) phdr->p_paddr,
							 phdr->p_filesz)) {
				free(ehdr);
				return -1;
			}
		}
	}

	free(ehdr);
	return 0;
}

/*
 * Dump crash to file
 */
static int dump_elf(Elf32_Ehdr *elfhdr_addr, char *filename)
{
	int fd;
	int rc;

	printf("Crash dump to %s\n", filename);
	fd = open_create(filename);
	if (fd < 0)
		return 1;
	rc = write_elf(elfhdr_addr, fd);
	close(fd);

	return rc;
}

/*
 * Wait for MMC/SD card to be inserted
 */
static int wait_for_mmc(void)
{
	struct mmc *mmc;

        mmc = find_mmc_device(CONFIG_MMC_DEV_NUM);
	if (!mmc) {
		printf("MMC device %d not found\n", CONFIG_MMC_DEV_NUM);
		return 1;
	}
	while (mmc_init(mmc) != 0) {
		kick_mpcore_wdt();
		printf("Insert MMC/SD card or press ctrl-c to abort\n");
		putc('.');
		udelay(500000);
		/* check for ctrl-c to abort... */
		if (ctrlc()) {
			puts("Abort\n");
			return -1;
		}
	}
	return 0;
}

/*
 * Find kexec/kdump ATAG command line
 */
static char *get_atag_cmdline(void)
{
	ulong atag_offset = 0x1000; /* 4k offset from memory start */
	ulong offset = 0x8000;      /* 32k offset from memory start */
	/*
	 * Get pointer to ATAG area, somewhere below U-boot image.
	 * Above values are hard coded in the kexec-tools.
	 */
	u32 * atags = (u32 *)(_armboot_start - offset + atag_offset);
	u32 i = 0;

	/*
	 * ATAG command line: \0 terminated string.
	 * The list ends with an ATAG_NONE node.
	 */
	for (i = 0; (atags[i] != 0) && (atags[i+1] != ATAG_NONE);
			i += atags[i]) {

		if (atags[i+1] == ATAG_CMDLINE)
			return (char *) &atags[i+2];
		/* sanity check before checking next ATAG */
		if (atags[i] > (offset - atag_offset) / sizeof(u32) - i)
			return NULL;
		if ((atags[i] + i) < i) /* cannot step backwards */
			return NULL;
	}

	return NULL;
}

/*
 * Find out where the kdump elf header is.
 */
static Elf32_Ehdr *get_elfhdr_addr(void)
{
	const char elfcorehdr[] = "elfcorehdr=";
	char *cmd;
	char *atag_cmdline = get_atag_cmdline();

	if (atag_cmdline != NULL) {
		cmd = strstr(atag_cmdline, elfcorehdr);
		if (cmd != NULL) {
			cmd += strlen(elfcorehdr);
			return (Elf32_Ehdr *) simple_strtoul(cmd, NULL, 16);
		}
	}
	return NULL;
}

/*
 * Dump crash to file (typically FAT file on SD/MMC).
 */
static int crashdump(Elf32_Ehdr *elfhdr_addr, char *filename)
{
	int rc;
	block_dev_desc_t *dev_desc=NULL;

	rc = wait_for_mmc();
	if (rc == 0) {
		dev_desc = get_dev("mmc", 1);	/* mmc 1 */
		rc = fat_register_device(dev_desc, 1); /* part 1 */
		if (rc != 0) {
			printf("crashdump: fat_register_device failed %d\n",
					rc);
			return -1;
		}
		rc = dump_elf(elfhdr_addr, filename);
	}

	if (rc != 0)
		printf("crashdump: error writing dump to %s\n", filename);

	return rc;
}

/*
 * Dump crash to file (typically FAT file on SD/MMC).
 */
static int do_checkcrash(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	Elf32_Ehdr *elfhdr_addr;
	int rc = -1;

	elfhdr_addr = get_elfhdr_addr();
	if (elfhdr_addr != NULL)
		rc = check_elfhdr(elfhdr_addr);
	if (rc == 0) {
		printf("crash dump elf header found. Dumping to card...\n");
		/* stop autoboot in case we were called by preboot */
		setenv("bootdelay", "-1");
		rc = crashdump(elfhdr_addr, crash_filename);
		if (rc != 0)
			printf("checkcrash: error writing dump from %x to %s\n"
			       , (u32) elfhdr_addr, crash_filename);
	}
	return rc;
}

U_BOOT_CMD(
	checkcrash,	1,	0,	do_checkcrash,
	"check ATAGS from crash and dump to file",
	"    - dump crash info to file and stop autoboot\n"
);
