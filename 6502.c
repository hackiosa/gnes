/*
 * Copyright (C) 2014 Frederic Meyer
 * 
 * This file is part of gnes.
 *
 * gnes is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *   
 * gnes is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gnes.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "6502.h"

uint8_t a, x, y, sp, p;
uint16_t pc;
uint8_t cycles;

/* This #define part is partially inspired by http://rubbermallet.org/fake6502.c */
#define FLAG_CARRY     0b00000001
#define FLAG_ZERO      0b00000010
#define FLAG_INTERRUPT 0b00000100
#define FLAG_BREAK     0b00010000
#define FLAG_OVERFLOW  0b00100000
#define FLAG_NEGATIVE  0b01000000

#define CARRY (p & FLAG_CARRY) // This is nice to have for instructions like ADC
#define ZERO (p & FLAG_ZERO)
#define OVERFLOW (p & FLAG_OVERFLOW)
#define NEGATIVE (p & FLAG_NEGATIVE)

#define set_carry()       p |=   FLAG_CARRY
#define clear_carry()     p &= (~FLAG_CARRY)
#define set_zero()        p |=   FLAG_ZERO
#define clear_zero()      p &= (~FLAG_ZERO)
#define set_interrupt()   p |=   FLAG_INTERRUPT
#define clear_interrupt() p &= (~FLAG_INTERRUPT)
#define set_break()       p |=   FLAG_BREAK
#define clear_break()     p &= (~FLAG_BREAK)
#define set_overflow()    p |=   FLAG_OVERFLOW
#define clear_overflow()  p &= (~FLAG_OVERFLOW)
#define set_negative()    p |=   FLAG_NEGATIVE
#define clear_negative()  p &= (~FLAG_NEGATIVE)

#define update_carry(n) {\
    if ((n) & 0xFF00) set_carry();\
        else clear_carry();\
}

#define update_zero(n) {\
    if ((n) & 0x00FF) clear_zero();\
        else set_zero();\
}

#define update_overflow(n, op1, op2) {\
    if ( ((op1) & 0x0080) == ((op2) & 0x0080) && ((op1) & 0x0080) != ((n) & 0x0080) ) set_overflow();\
        else clear_overflow();\
}

#define update_negative(n) {\
    if ((n) & 0x0080) set_negative();\
        else clear_negative();\
}

extern uint8_t mmu_read(uint16_t addr);
extern uint16_t mmu_read16(uint16_t addr);
extern void mmu_write(uint16_t addr, uint8_t value);
extern void mmu_write16(uint16_t addr, uint16_t value);

void cpu_push(uint8_t value)
{
    mmu_write(0x0100 + sp, value);
    sp--;
}

void cpu_push16(uint16_t value)
{
    mmu_write16(0x0100 + sp - 1, value);
    sp -= 2;
}

uint8_t cpu_pop()
{
    sp++;
    return mmu_read(0x0100 + sp);
}

uint16_t cpu_pop16()
{
    uint16_t tmp = mmu_read16(sp + 1);
    sp += 2;
    return tmp;
}

/* ADC: Fix cycles for specific opcodes ON Page Cross */
void adc_imm()
{
    uint8_t value = mmu_read(pc + 1);
    uint16_t result = a + value + CARRY;
    update_overflow(result, a, value + CARRY);
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc += 2;
}

void adc_zp()
{
    uint8_t value = mmu_read(mmu_read(pc + 1));
    uint16_t result = a + value + CARRY;
    update_overflow(result, a, value + CARRY);
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc += 2;
}

void adc_zpx()
{
    uint8_t value = mmu_read((mmu_read(pc + 1) + x) & 0xFF);
    uint16_t result = a + value + CARRY;
    update_overflow(result, a, value + CARRY);
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc += 2;
}

void adc_abs()
{
    uint8_t value = mmu_read(mmu_read16(pc + 1));
    uint16_t result = a + value + CARRY;
    update_overflow(result, a, value + CARRY);
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc += 3;
}

void adc_abx()
{
    uint8_t value = mmu_read(mmu_read16(pc + 1) + x);
    uint16_t result = a + value + CARRY;
    update_overflow(result, a, value + CARRY);
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc += 3;
}

void adc_aby()
{
    uint8_t value = mmu_read(mmu_read16(pc + 1) + y);
    uint16_t result = a + value + CARRY;
    update_overflow(result, a, value + CARRY);
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc += 3;
}

void adc_idx()
{
    uint8_t value = mmu_read(mmu_read16(mmu_read(pc + 1) + x));
    uint16_t result = a + value + CARRY;
    update_overflow(result, a, value + CARRY);
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc += 2;
}

void adc_idy()
{
    uint8_t value = mmu_read(mmu_read16(mmu_read(pc + 1)) + y);
    uint16_t result = a + value + CARRY;
    update_overflow(result, a, value + CARRY);
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc += 2;
}

/* AND: Fix cycles for specific opcodes ON Page Cross */
void and_imm()
{
    int8_t value = mmu_read(pc + 1);
    uint8_t result = a & value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 2;
}

void and_zp()
{
    uint8_t value = mmu_read(mmu_read(pc + 1));
    uint8_t result = a & value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 2;
}

void and_zpx()
{
    uint8_t value = mmu_read((mmu_read(pc + 1) + x) & 0xFF);
    uint8_t result = a & value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 2;
}

void and_abs()
{
    uint8_t value = mmu_read(mmu_read16(pc + 1));
    uint8_t result = a & value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 3;
}

void and_abx()
{
    uint8_t value = mmu_read(mmu_read16(pc + 1) + x);
    uint8_t result = a & value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 3;
}

void and_aby()
{
    uint8_t value = mmu_read(mmu_read16(pc + 1) + y);
    uint8_t result = a & value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 3;
}

void and_idx()
{
    uint8_t value = mmu_read(mmu_read16(mmu_read(pc + 1) + x));
    uint8_t result = a & value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 2;
}

void and_idy()
{
    uint8_t value = mmu_read(mmu_read16(mmu_read(pc + 1)) + y);
    uint8_t result = a & value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 2;
}

void asl_acc()
{
    uint16_t result = a << 1;
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc++;
}

void asl_zp()
{
    uint16_t result = mmu_read(mmu_read(pc + 1)) << 1;
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc += 2;
}

void asl_zpx()
{
    uint16_t result = mmu_read((mmu_read(pc + 1) + x) & 0xFF) << 1;
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc += 2;
}

void asl_abs()
{
    uint16_t result = mmu_read(mmu_read16(pc + 1)) << 1;
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc += 3;
}

void asl_abx()
{
    uint16_t result = mmu_read(mmu_read16(pc + 1) + x) << 1;
    update_carry(result);
    update_zero(result);
    update_negative(result);
    a = result & 0xFF;
    pc += 3;
}

void bcc_rel()
{
    if (!CARRY)
    {
        uint16_t oldpc = pc;
        uint16_t rel = (uint16_t)mmu_read(pc + 1);
        if (rel & 0x80) rel |= 0xFF00;
        pc += rel;
        if ((oldpc & 0xFF00) == (pc & 0xFF00))
            cycles += 2;
        else
            cycles += 1; 
        return;
    }
    pc += 2;
}

void bcs_rel()
{
    if (CARRY)
    {
        uint16_t oldpc = pc;
        uint16_t rel = (uint16_t)mmu_read(pc + 1);
        if (rel & 0x80) rel |= 0xFF00;
        pc += rel;
        if ((oldpc & 0xFF00) == (pc & 0xFF00))
            cycles += 2;
        else
            cycles += 1; 
        return;
    }
    pc += 2;
}

void beq_rel()
{
    if (ZERO)
    {
        uint16_t oldpc = pc;
        uint16_t rel = (uint16_t)mmu_read(pc + 1);
        if (rel & 0x80) rel |= 0xFF00;
        pc += rel;
        if ((oldpc & 0xFF00) == (pc & 0xFF00))
            cycles += 2;
        else
            cycles += 1; 
        return;
    }
    pc += 2;
}

void bit_zp()
{
    uint8_t value = mmu_read(mmu_read(pc + 1));
    uint8_t result = a & value;
    update_zero(result);
    update_negative(value);
    if (value & 0x40) set_overflow();
        else clear_overflow();
    pc += 2;
}

void bit_abs()
{
    uint8_t value = mmu_read(mmu_read16(pc + 1));
    uint8_t result = a & value;
    update_zero(result);
    update_negative(value);
    if (value & 0x40) set_overflow();
        else clear_overflow();
    pc += 3;
}

void bmi_rel()
{
    if (NEGATIVE)
    {
        uint16_t oldpc = pc;
        uint16_t rel = (uint16_t)mmu_read(pc + 1);
        if (rel & 0x80) rel |= 0xFF00;
        pc += rel;
        if ((oldpc & 0xFF00) == (pc & 0xFF00))
            cycles += 2;
        else
            cycles += 1; 
        return;
    }
    pc += 2;
}

void bne_rel()
{
    if (!ZERO)
    {
        uint16_t oldpc = pc;
        uint16_t rel = (uint16_t)mmu_read(pc + 1);
        if (rel & 0x80) rel |= 0xFF00;
        pc += rel;
        if ((oldpc & 0xFF00) == (pc & 0xFF00))
            cycles += 2;
        else
            cycles += 1; 
        return;
    }
    pc += 2;
}

void bpl_rel()
{
    if (!NEGATIVE)
    {
        uint16_t oldpc = pc;
        uint16_t rel = (uint16_t)mmu_read(pc + 1);
        if (rel & 0x80) rel |= 0xFF00;
        pc += rel;
        if ((oldpc & 0xFF00) == (pc & 0xFF00))
            cycles += 2;
        else
            cycles += 1; 
        return;
    }
    pc += 2;
}

void brk()
{
    cpu_push16(pc + 2);
    cpu_push(p | FLAG_BREAK);
    set_interrupt();
    pc = mmu_read16(0xFFFE);     
}

void bvc_rel()
{
    if (!OVERFLOW)
    {
        uint16_t oldpc = pc;
        uint16_t rel = (uint16_t)mmu_read(pc + 1);
        if (rel & 0x80) rel |= 0xFF00;
        pc += rel;
        if ((oldpc & 0xFF00) == (pc & 0xFF00))
            cycles += 2;
        else
            cycles += 1; 
        return;
    }
    pc += 2;
}

void bvs_rel()
{
    if (OVERFLOW)
    {
        uint16_t oldpc = pc;
        uint16_t rel = (uint16_t)mmu_read(pc + 1);
        if (rel & 0x80) rel |= 0xFF00;
        pc += rel;
        if ((oldpc & 0xFF00) == (pc & 0xFF00))
            cycles += 2;
        else
            cycles += 1; 
        return;
    }
    pc += 2;
}

void clc()
{
    clear_carry();
    pc++;
}

void cli()
{
    clear_interrupt();
    pc++;
}

void clv()
{
    clear_overflow();
    pc++;
}

void cmp_imm()
{
    uint8_t value = mmu_read(pc + 1);
    if (a >= value) set_carry();
        else clear_carry();
    update_zero(a - value);
    update_negative(a - value);
    pc += 2;
}

void cmp_zp()
{
    uint8_t value = mmu_read(mmu_read(pc + 1));
    if (a >= value) set_carry();
        else clear_carry();
    update_zero(a - value);
    update_negative(a - value);
    pc += 2;
}

void cmp_zpx()
{
    uint8_t value = mmu_read((mmu_read(pc + 1) + x) & 0xFF);
    if (a >= value) set_carry();
        else clear_carry();
    update_zero(a - value);
    update_negative(a - value);
    pc += 2;
}

void cmp_abs()
{
    uint8_t value = mmu_read(mmu_read16(pc + 1));
    if (a >= value) set_carry();
        else clear_carry();
    update_zero(a - value);
    update_negative(a - value);
    pc += 3;
}

void cmp_abx()
{
    uint16_t address = mmu_read16(pc + 1) + x;
    uint8_t value = mmu_read(address);
    if (a >= value) set_carry();
        else clear_carry();
    update_zero(a - value);
    update_negative(a - value);
    if ((address & 0xFF00) != (pc & 0xFF00))
        cycles += 1;
    pc += 3;
}

void cmp_aby()
{
    uint16_t address = mmu_read16(pc + 1) + y;
    uint8_t value = mmu_read(address);
    if (a >= value) set_carry();
        else clear_carry();
    update_zero(a - value);
    update_negative(a - value);
    if ((address & 0xFF00) != (pc & 0xFF00))
        cycles += 1;
    pc += 3;
}

void cmp_idx()
{
    uint8_t value = mmu_read(mmu_read16(mmu_read(pc + 1) + x));
    if (a >= value) set_carry();
        else clear_carry();
    update_zero(a - value);
    update_negative(a - value);
    pc += 2;
}

void cmp_idy()
{
    uint16_t address = mmu_read16(mmu_read(pc + 1)) + y;
    uint8_t value = mmu_read(address);
    if (a >= value) set_carry();
        else clear_carry();
    update_zero(a - value);
    update_negative(a - value);
    if ((address & 0xFF00) != (pc & 0xFF00))
        cycles += 1;
    pc += 2;
}

void cpx_imm()
{
    uint8_t value = mmu_read(pc + 1);
    if (x >= value) set_carry();
        else clear_carry();
    update_zero(x - value);
    update_negative(x - value);
    pc += 2;
}

void cpx_zp()
{
    uint8_t value = mmu_read(mmu_read(pc + 1));
    if (x >= value) set_carry();
        else clear_carry();
    update_zero(x - value);
    update_negative(x - value);
    pc += 2;
}

void cpx_abs()
{
    uint8_t value = mmu_read(mmu_read16(pc + 1));
    if (x >= value) set_carry();
        else clear_carry();
    update_zero(x - value);
    update_negative(x - value);
    pc += 3;
}

void cpy_imm()
{
    uint8_t value = mmu_read(pc + 1);
    if (x >= value) set_carry();
        else clear_carry();
    update_zero(x - value);
    update_negative(x - value);
    pc += 2;
}

void cpy_zp()
{
    uint8_t value = mmu_read(mmu_read(pc + 1));
    if (x >= value) set_carry();
        else clear_carry();
    update_zero(x - value);
    update_negative(x - value);
    pc += 2;
}

void cpy_abs()
{
    uint8_t value = mmu_read(mmu_read16(pc + 1));
    if (x >= value) set_carry();
        else clear_carry();
    update_zero(x - value);
    update_negative(x - value);
    pc += 3;
}

void dec_zp()
{
    uint16_t address = mmu_read(pc + 1);
    uint8_t result = mmu_read(address) - 1;
    update_zero(result);
    update_negative(result);
    mmu_write(address, result);
    pc += 2;
}

void dec_zpx()
{
    uint16_t address = (mmu_read(pc + 1) + x) & 0xFF;
    uint8_t result = mmu_read(address) - 1;
    update_zero(result);
    update_negative(result);
    mmu_write(address, result);
    pc += 2;
}

void dec_abs()
{
    uint16_t address = mmu_read16(pc + 1);
    uint8_t result = mmu_read(address) - 1;
    update_zero(result);
    update_negative(result);
    mmu_write(address, result);
    pc += 3;
}

void dec_abx()
{
    uint16_t address = mmu_read16(pc + 1) + x;
    uint8_t result = mmu_read(address) - 1;
    update_zero(result);
    update_negative(result);
    mmu_write(address, result);
    pc += 3;
}

void dex()
{
    uint8_t result = x - 1;
    update_zero(result);
    update_negative(result);
    x = result;
    pc++;
}

void dey()
{
    uint8_t result = x - 1;
    update_zero(result);
    update_negative(result);
    x = result;
    pc++;
}

void eor_imm()
{
    uint8_t value = mmu_read(pc + 1);
    uint8_t result = a ^ value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 2;
}

void eor_zp()
{
    uint8_t value = mmu_read(mmu_read(pc + 1));
    uint8_t result = a ^ value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 2;
}

void eor_zpx()
{
    uint8_t value = mmu_read((mmu_read(pc + 1) + x) & 0xFF);
    uint8_t result = a ^ value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 2;
}

void eor_abs()
{
    uint8_t value = mmu_read(mmu_read16(pc + 1));
    uint8_t result = a ^ value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 3;    
}

void eor_abx()
{
    uint16_t address = mmu_read16(pc + 1) + x;
    uint8_t value = mmu_read(address);
    uint8_t result = a ^ value;
    update_zero(result);
    update_negative(result);
    a = result;
    if ((address & 0xFF00) != (pc & 0xFF00))
        cycles += 1;
    pc += 3;    
}

void eor_aby()
{
    uint16_t address = mmu_read16(pc + 1) + y;
    uint8_t value = mmu_read(address);
    uint8_t result = a ^ value;
    update_zero(result);
    update_negative(result);
    a = result;
    if ((address & 0xFF00) != (pc & 0xFF00))
        cycles += 1;
    pc += 3;    
}

void eor_idx()
{
    uint8_t value = mmu_read(mmu_read16(mmu_read(pc + 1) + x));
    uint8_t result = a ^ value;
    update_zero(result);
    update_negative(result);
    a = result;
    pc += 2; 
}

void eor_idy()
{
    uint16_t address = mmu_read16(mmu_read(pc + 1)) + y;
    uint8_t value = mmu_read(address);
    uint8_t result = a ^ value;
    update_zero(result);
    update_negative(result);
    a = result;
    if ((address & 0xFF00) != (pc & 0xFF00))
        cycles += 1;
    pc += 2; 
}

void inc_zp()
{
    uint8_t address = mmu_read(pc + 1);
    uint8_t result = mmu_read(address) + 1;
    update_zero(result);
    update_negative(result);
    mmu_write(address, result);
    pc += 2;
}

void inc_zpx()
{
    uint8_t address = (mmu_read(pc + 1) + x) & 0xFF;
    uint8_t result = mmu_read(address) + 1;
    update_zero(result);
    update_negative(result);
    mmu_write(address, result);
    pc += 2;
}

void inc_abs()
{
    uint8_t address = mmu_read16(pc + 1);
    uint8_t result = mmu_read(address) + 1;
    update_zero(result);
    update_negative(result);
    mmu_write(address, result);
    pc += 3;
}

void inc_abx()
{
    uint8_t address = mmu_read16(pc + 1) + x;
    uint8_t result = mmu_read(address) + 1;
    update_zero(result);
    update_negative(result);
    mmu_write(address, result);
    pc += 3;
}

void inx()
{
    x++;
    update_zero(x);
    update_negative(x);
    pc++;
}

void iny()
{
    y++;
    update_zero(y);
    update_negative(y);
    pc++;
}

void jmp_abs()
{
    pc = mmu_read16(pc + 1);
}

void jmp_ind()
{
    uint16_t pointer = mmu_read16(pc + 1);
    if (pointer & 0xFF) // Target address is not correctly read if the indirect vector falls on a page boundary
    {
        pc = mmu_read(pointer) | (mmu_read(pointer & 0xFF00) << 8); 
    } else {
        pc = mmu_read16(pointer);
    }
}

void jsr_abs()
{
    cpu_push16(pc + 2);
    pc = mmu_read(pc + 1);
}

void cpu_step()
{
    switch (mmu_read(pc))
    {
    case 0x00: brk(); break;
    case 0x06: asl_zp(); break;
    case 0x0A: asl_acc(); break;
    case 0x0E: asl_abs(); break;
    case 0x10: bpl_rel(); break;
    case 0x16: asl_zpx(); break;
    case 0x18: clc(); break;
    case 0x1E: asl_abx(); break;
    case 0x20: jsr_abs(); break;
    case 0x21: and_idx(); break;
    case 0x24: bit_zp(); break;
    case 0x25: and_zp(); break;
    case 0x29: and_imm(); break;
    case 0x2C: bit_abs(); break;
    case 0x2D: and_abs(); break;
    case 0x30: bmi_rel(); break;
    case 0x31: and_idy(); break;
    case 0x35: and_zpx(); break;
    case 0x39: and_aby(); break;
    case 0x3D: and_abx(); break;
    case 0x41: eor_idx(); break;
    case 0x45: eor_zp(); break;
    case 0x49: eor_imm(); break;
    case 0x4C: jmp_abs(); break;
    case 0x4D: eor_abs(); break;
    case 0x50: bvc_rel(); break;
    case 0x51: eor_idy(); break;
    case 0x55: eor_zpx(); break;
    case 0x58: cli(); break;
    case 0x59: eor_aby(); break;
    case 0x5D: eor_abx(); break;
    case 0x61: adc_idx(); break;
    case 0x65: adc_zp(); break;
    case 0x69: adc_imm(); break;
    case 0x6C: jmp_ind(); break;
    case 0x6D: adc_abs(); break;
    case 0x70: bvs_rel(); break;
    case 0x71: adc_idy(); break;
    case 0x75: adc_zpx(); break;
    case 0x79: adc_aby(); break;
    case 0x7D: adc_abx(); break;
    case 0x88: dey(); break;
    case 0x90: bcc_rel(); break;
    case 0xB0: bcs_rel(); break;
    case 0xB8: clv(); break;
    case 0xC0: cpy_imm(); break;
    case 0xC1: cmp_idx(); break;
    case 0xC4: cpy_zp(); break;
    case 0xC5: cmp_zp(); break;
    case 0xC6: dec_zp(); break;
    case 0xC8: iny(); break;
    case 0xC9: cmp_imm(); break;
    case 0xCA: dex(); break;
    case 0xCC: cpy_abs(); break;
    case 0xCD: cmp_abs(); break;
    case 0xCE: dec_abs(); break;
    case 0xD0: bne_rel(); break;
    case 0xD1: cmp_idy(); break;
    case 0xD5: cmp_zpx(); break;
    case 0xD6: dec_zpx(); break;
    case 0xD9: cmp_aby(); break;
    case 0xDD: cmp_abx(); break;
    case 0xDE: dec_abx(); break;
    case 0xE0: cpx_imm(); break;
    case 0xE4: cpx_zp(); break;
    case 0xE6: inc_zp(); break;
    case 0xE8: inx(); break;
    case 0xEC: cpx_abs(); break;
    case 0xEE: inc_abs(); break;
    case 0xF0: beq_rel(); break;
    case 0xF6: inc_zpx(); break;
    case 0xFE: inc_abx(); break;
    default:
        printf("6502.c: Meeh! I don't know that instruction @ %4x\n", pc);
        break;
    }
}
