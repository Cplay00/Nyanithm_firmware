#!/usr/bin/env python3
"""ELF32 to UF2 converter for RP2040 (replaces picotool uf2 convert).
Based on picotool's elf2uf2.cpp algorithm."""

import struct
import sys

UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END    = 0x0AB16F30
UF2_FLAG_FAMILY_ID_PRESENT = 0x00002000
RP2040_FAMILY_ID = 0xe48bff56
UF2_PAGE_SIZE = 256
FLASH_SECTOR_ERASE_SIZE = 4096

# RP2040 address ranges
MAIN_FLASH_START = 0x10000000
MAIN_FLASH_END   = 0x11000000

def parse_elf32(data):
    """Parse ELF32 header and program headers."""
    # ELF32 header
    e_ident = data[:16]
    if e_ident[:4] != b'\x7fELF':
        raise ValueError("Not an ELF file")
    ei_class = e_ident[4]  # 1 = 32-bit
    if ei_class != 1:
        raise ValueError("Not ELF32")
    (e_type, e_machine, e_version, e_entry,
     e_phoff, e_shoff, e_flags, e_ehsize,
     e_phentsize, e_phnum) = struct.unpack_from('<HHIIIIIHHH', data, 16)

    phdrs = []
    for i in range(e_phnum):
        off = e_phoff + i * e_phentsize
        (p_type, p_offset, p_vaddr, p_paddr,
         p_filesz, p_memsz, p_flags, p_align) = struct.unpack_from('<IIIIIIII', data, off)
        phdrs.append({
            'type': p_type, 'offset': p_offset, 'vaddr': p_vaddr,
            'paddr': p_paddr, 'filesz': p_filesz, 'memsz': p_memsz,
            'flags': p_flags, 'align': p_align
        })
    return e_entry, phdrs

def check_flash_range(paddr, size):
    """Check if physical address is in RP2040 flash range."""
    return (paddr >= MAIN_FLASH_START and paddr + size <= MAIN_FLASH_END)

def elf2uf2(elf_path, uf2_path):
    with open(elf_path, 'rb') as f:
        elf_data = f.read()

    entry, phdrs = parse_elf32(elf_data)

    # Build page map: page_addr -> list of (file_offset, page_offset, bytes)
    pages = {}
    PT_LOAD = 1
    for ph in phdrs:
        if ph['type'] != PT_LOAD or ph['memsz'] == 0:
            continue
        mapped_size = min(ph['filesz'], ph['memsz'])
        if mapped_size == 0:
            continue
        if not check_flash_range(ph['paddr'], mapped_size):
            continue
        addr = ph['paddr']
        remaining = mapped_size
        file_offset = ph['offset']
        while remaining > 0:
            off = addr & (UF2_PAGE_SIZE - 1)
            length = min(remaining, UF2_PAGE_SIZE - off)
            page_addr = addr - off
            if page_addr not in pages:
                pages[page_addr] = []
            pages[page_addr].append((file_offset, off, length))
            addr += length
            file_offset += length
            remaining -= length

    if not pages:
        raise ValueError("No flash pages found in ELF")

    # Pad to flash sector boundaries (except last sector)
    sorted_pages = sorted(pages.keys())
    last_page = sorted_pages[-1]
    all_page_addrs = set(sorted_pages)
    for pa in list(sorted_pages):
        sector = pa // FLASH_SECTOR_ERASE_SIZE
        for page in range(sector * FLASH_SECTOR_ERASE_SIZE,
                          (sector + 1) * FLASH_SECTOR_ERASE_SIZE,
                          UF2_PAGE_SIZE):
            if page < last_page and page not in all_page_addrs:
                pages[page] = []  # empty = zero-filled padding
                all_page_addrs.add(page)

    # Sort final pages
    final_pages = sorted(pages.items())
    total_blocks = len(final_pages)

    # Write UF2
    with open(uf2_path, 'wb') as out:
        for block_no, (page_addr, fragments) in enumerate(final_pages):
            # Build 256-byte data page
            page_data = bytearray(UF2_PAGE_SIZE)
            for (f_off, p_off, nbytes) in fragments:
                page_data[p_off:p_off+nbytes] = elf_data[f_off:f_off+nbytes]

            # UF2 block = 512 bytes
            block = struct.pack('<IIIIIIII',
                UF2_MAGIC_START0,
                UF2_MAGIC_START1,
                UF2_FLAG_FAMILY_ID_PRESENT,  # flags
                page_addr,                    # target_addr
                UF2_PAGE_SIZE,                # payload_size
                block_no,                     # block_no
                total_blocks,                 # num_blocks
                RP2040_FAMILY_ID,             # file_size / family_id
            )
            block += bytes(page_data)                    # data[256]
            block += b'\x00' * (476 - UF2_PAGE_SIZE)     # padding to 476
            block += struct.pack('<I', UF2_MAGIC_END)     # magic_end
            assert len(block) == 512
            out.write(block)

    print(f"Converted {elf_path} -> {uf2_path}")
    print(f"  Pages: {total_blocks}, UF2 size: {total_blocks * 512} bytes")
    print(f"  Flash range: 0x{final_pages[0][0]:08X} - 0x{final_pages[-1][0]:08X}")
    return total_blocks

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input.elf> <output.uf2>")
        sys.exit(1)
    # round73: 路径规范化与校验——resolve() 消除 '../' 歧义;输入必须是已存在的
    # .elf 文件,输出必须是 .uf2 目标。不限制所在驱动器/目录: 构建脚本按设计会把
    # UF2 写到仓库外(如 D:\Nyanithm_build\fw_v1\build\)。
    from pathlib import Path
    elf_in = Path(sys.argv[1]).resolve()
    uf2_out = Path(sys.argv[2]).resolve()
    if elf_in.suffix.lower() != '.elf' or not elf_in.is_file():
        print(f"ERROR: input must be an existing .elf file: {elf_in}")
        sys.exit(1)
    if uf2_out.suffix.lower() != '.uf2':
        print(f"ERROR: output must be a .uf2 path: {uf2_out}")
        sys.exit(1)
    elf2uf2(str(elf_in), str(uf2_out))
