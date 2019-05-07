@------------------
#define REGBASE_IO        0x4000000
#define REG_DMA3_SAD      0x0D4
#define REG_DMA3_DAD              0x0D8
#define REG_DMA3_CNT              0x0DC
#define REG_DMA3_FILL             0x0EC
@---
#define REGBASE_SCFG              0x4004000
#define REG_SCFG_ROM              0x000
#define REG_SCFG_CLK7             0x004
#define REG_SCFG_CLK9             0x004
#define REG_SCFG_EXT7             0x008
#define REG_SCFG_EXT9             0x008
@---
#define REGBASE_NDMA              0x4004100
@---
#define REGBASE_SDIO              0x4004A00
#define REG_SDIO_CMD              0x000
#define REG_SDIO_CARD_PORT_SELECT 0x002
#define REG_SDIO_CMD_PARAM        0x004
#define REG_SDIO_STOP_INTERNAL    0x008
#define REG_SDIO_NUMBLK16         0x00A
#define REG_SDIO_REPLY            0x00C
#define REG_SDIO_IRQ_STAT         0x01C
#define REG_SDIO_CARD_CLK_CTL     0x024
#define REG_SDIO_BLKLEN16         0x026
#define REG_SDIO_CARD_OPTION      0x028
#define REG_SDIO_DATA16           0x030
#define REG_SDIO_CARD_IRQ_ENABLE  0x034
#define REG_SDIO_CARD_IRQ_STAT    0x036
#define REG_SDIO_CARD_IRQ_DISABLE 0x038
#define REG_SDIO_DATA_CTL         0x0D8
#define REG_SDIO_SOFT_RESET       0x0E0
#define REG_SDIO_IRQ32            0x100
#define REG_SDIO_BLKLEN32         0x104
#define REG_SDIO_NUMBLK32         0x108
#define REG_SDIO_DATA32           0x10C
@---
#define REGBASE_GPIO              0x4004C00
#define REG_GPIO_WIFI             0x004
#define REG_SPI_CNT_DATA          0x1C0  
                                        @(1C0h=cnt, and 1C2h=data)
#define REG_SPI_CNT               0x1C0
#define REG_SPI_DATA              0x1C2
#define REG_IME                   0x208

#define with_dsi_wifi             1
#define wifi_with_dsi_irq         1
#define try_sdio_ndma             1
#define try_fast_buffer           1
#define use_dsi_bios              0

@:----------------- DSi SDIO
@------------------
.if with_dsi_wifi
@------------------
#define verbose_wifi_firmware       0
#define try_sdio_data32_mode        0  @not working?
#define with_rtc_init               0  @already done by launcher/unlaunch
#define with_wifi_firmware_loading  0  @already done by launcher/unlaunch
#define with_manual_wifi_decompress 0
#define sdio_dma                    0 @1    @XXX requires DMA channel to be free/unused
                                            @XXX above DMA doesn't work yet with TX multiblock (len>80h)
.endif

@---
#define swi_wait_by_loop      0x03
#define swi_crc16             0x0E

@---
@------------------
firmware_get_send:  @in: r2=len=1=last, io: r0=data
 push  {r3-r4,lr}
 mov   r3,#0x8100     @ctrl firmware, 4MHz
spi_get_send_inj:
 mov   r4,#0x4000000  @iobase     @-I/O Base
 strb  r4,[r4,#REG_IME] @IME=0   @-ensure no IRQs during SPI access (*)
 cmp   r2,#1         @len=1=last @\
 orrne r3,#0x0800     @want more  @ SPI Control and Data Out
 orr   r0,r3,r0,lsl #16          @
 str   r0,[r4,#REG_SPI_CNT_DATA] @/
wait:                         @\
 ldrb  r0,[r4,REG_SPI_CNT]      @ Wait Busy
 tst   r0,#0x80       @busy       @
 bne   wait                   @/
 cmp   r2,#1         @len=1=last @\
 moveq r0,#3    @WaitByLoop      @ Extra Delay after last byte
 swieq #0x03 << 16 @wait_by_loop @/ <-- crashes only r0
 ldrb  r0,[r4,#REG_SPI_DATA]     @-Data in
 cmp   r2,#1         @len=1=last @\
 streqb r2,[r4,#REG_IME] @IME=1  @/ <-- resurrect IRQs after SPI access (*)
 subs  r2,#1         @len        @-Decrease data (out: z=last)
 pop   {r3-r4,pc}     @out: zf
 @---

@------------------
firmware_send_cmd_and_adr:  @in: r2=len, r3=cmd/addr
 push {r0-r7,lr}
 add  r2,#4  @dta+cmdlen @\
 mov  r6,#24 @bitpos     @
send_lop:             @ send command-byte and 3-byte-address
 mov  r0,r3,lsr r6      @
 bl   firmware_get_send @
 subs r6,#8 @bitpos      @
 bpl  send_lop        @/
 pop  {r0-r7,pc}
@------------------
arm7_readFirmware:  @in: r0=src, r1=dst, r2=len
 push {r0-r12,lr}
 bic  r0, #0x0ff000000             @-truncate 32bit to 24bit address
 orr  r3,r0,#0x03 << 24          @-read command & source address
 bl   firmware_send_cmd_and_adr @-send command/address
read_lop:                     @\
 bl   firmware_get_send         @ read block
 strb r0,[r1],#1                 @
 bne  read_lop                @/
 pop  {r0-r12,pc}
@------------------
sdio_atheros_init:
 push {lr}
 .if verbose_wifi_firmware
   bl   get_restart_timer         @-
   mov  r0,chr_cls                @\
   call wrchr_r0                  @/
   mov  r0,chr_gif                @\ATHEROS: request text+gif layer enable
   bl   wrchr_r0                  @/
 .endif
@- - -
 ldr  r0,=0x1FD                    @src  @\
 ldr  r1,=twlcfg_etc_buf+0x1E0     @dst  @ load SPI bus FLASH wifi board version
 mov  r2,#1                        @len  @
 bl   arm7_readFirmware                 @/
@- - -
 bl   sdio_controller_init                              @-init controller
 bl   sdio_init_opcond_if_needed                        @\init device
 mov  r8,r0  @need_upload                               @/
 bl   sdio_init_func0                                   @-init func0
 movs r0,r8  @skip below when already NEEDING upload    @\
 bleq sdio_check_host_interest                          @ check RAM
 mov  r8,r0  @need_upload (if it got set by above)      @/
 bl   sdio_reset                                        @-reset
 .if with_wifi_firmware_loading
   cmp  r8,0   @need_upload                             @\load firmware
   blne sdio_load_firmware                              @/
 .endif
 bl   sdio_bmi_init                                     @-BMI init
 .if with_wifi_firmware_loading
   cmp  r8,0   @need_upload                             @\BMI upload
   blne sdio_bmi_upload_firmware                        @/
 .endif
 bl   sdio_bmi_finish                                   @-BMI finish
 bl   sdio_get_eeprom_stuff                             @-get EEPROM
 bl   sdio_whatever_handshake                           @-handshake
@- - -
 .if with_wifi_firmware_loading
   ldr  r0,=sdio_wmi_7th_cmd              @\WMI 7th command (WMI_SET_SCAN_PARAMS_CMD)
   bl   sdio_send_mbox_block              @/
   @- - -
   ldr r1,=418h // mov r0,00000000h // bl sdio_write_func1word  @-INT_STATUS_ENABLE (all back off)
   bl   sdio_check_mbox_state                                   @-check (but get nothing)
   ldr r1,=400h // ldr r0,=0040001h // bl sdio_write_func1word  @-[1:00400]=00040001h    @.......... (ack above read check_state [400h] value?)
   bl   sdio_recv_mbox_block                                    @-RECV (handshake)
   bl   sdio_check_mbox_state                                   @-check (but get nothing)
   ldr r1,=400h // mov r0,00040000h // bl sdio_write_func1word  @-[1:00400]=00040000h    @.......... (ack above read check_state [400h] value?)
   mov r1,0004h // mov r0,000h // bl sdio_write_func0byte       @-CCCR interrupt_enable (disable)
 .endif
@- - -
 .if verbose_wifi_firmware
  ldr  r1,=txt_wifi_done                @\
  bl   wrstr_r1                         @/
  bl   show_timer                       @-
  bl   flushkey_waitkey
 .endif
 pop  {pc}
@------------------
const_00000002h : .word 0x00000002
const_00000080h : .word 0x00000080
const_00000063h : .word 0x00000063
@---
sdio_mbox_1st_handshake : .byte 0,0,0x08,0,0,0,2,0, 0,1,0,0,0,0    @send cmd_2_2_1      @WMI_CONTROL ?            @\
sdio_mbox_2nd_handshake : .byte 0,0,0x08,0,0,0,2,0, 1,1,5,0,0,0    @send cmd_2_2_1_1_5  @WMI_DATA_BE best effort? @
sdio_mbox_3rd_handshake : .byte 0,0,0x08,0,0,0,2,0, 2,1,5,0,0,0    @send cmd_2_2_2_1_5  @WMI_DATA_BK background?  @ named so in launcher
sdio_mbox_4th_handshake : .byte 0,0,0x08,0,0,0,2,0, 3,1,5,0,0,0    @send cmd_2_2_3_1_5  @WMI_DATA_VI video?       @
sdio_mbox_5th_handshake : .byte 0,0,0x08,0,0,0,2,0, 4,1,5,0,0,0    @send cmd_2_2_4_1_5  @WMI_DATA_VO voice?       @/
sdio_mbox_6th_handshake : .byte 0,0,0x02,0,0,0,4,0                 @send cmd_4
sdio_wmi_7th_cmd        : .byte 1,1,0x16,0,0,0,8,0, 0x0ff,0x0ff,0x0ff,0x0ff, 0x0ff,0x0ff,0x14,0, 0x32,0,3,0, 0,0,0,0, 0,0,0,0 @send cmd_1_1_16_8_xxx (!) WMI_SET_SCAN_PARAMS_CMD
.align 4
@------------------
sdio_controller_init:
 push {lr}
 .if with_rtc_init
                @REQUIRED for sdio/wifi after BATTERY REMOVAL
                 bl   nds7_rtc_select           @-begin
                 mov  r0,48h+06h @write+idcode  @\send write command, index, idcode
                 bl   nds7_rtc_send_byte_r0     @/     FOUT register setting 1
                 mov  r0,80h                    @\data block
                 bl   nds7_rtc_send_byte_r0     @/
                 bl   nds7_rtc_deselect         @-finish
                @- - -
                 bl   nds7_rtc_select           @-begin
                 mov  r0,28h+06h @write+idcode  @\send write command, index, idcode
                 bl   nds7_rtc_send_byte_r0     @/     FOUT register setting 2
                 mov  r0,00h                    @\data block
                 bl   nds7_rtc_send_byte_r0     @/
                 bl   nds7_rtc_deselect         @-finish
 .endif
 @- - -
 ldr  r9,=REGBASE_SDIO
 @---part1
 ldrh r0,[r9,#REG_SDIO_SOFT_RESET]       @\
 bic  r0,#3                              @ SDIO_SOFT_RESET clear bit0-1
 strh r0,[r9,#REG_SDIO_SOFT_RESET]       @/
 ldrh r0,[r9,#REG_SDIO_SOFT_RESET]       @\
 orr  r0,#3                              @ SDIO_SOFT_RESET set bit0-1
 strh r0,[r9,#REG_SDIO_SOFT_RESET]       @/
 ldrh r0,[r9,#REG_SDIO_STOP_INTERNAL]    @\
        ORR r0,#0x100  @<-- bit8 needed for SDIO with multiblock !!!
 bic  r0,#1                              @ SDIO_STOP_INTERNAL clear bit0
 strh r0,[r9,#REG_SDIO_STOP_INTERNAL]    @/

@mov  r0,N/A                            @\SDIO_CARD_PORT_SELECT (uninit by firmware?)
@strh r0,[r9,REG_SDIO_CARD_PORT_SELECT] @/

 ldr  r0,=0x80d0                         @\SDIO_CARD_OPTION = 80D0h
 strh r0,[r9,#REG_SDIO_CARD_OPTION]      @/
 @---part2a
 mov  r0,#0x0040                          @\SDIO_CARD_CLK_CTL = 0040h
 strh r0,[r9,#REG_SDIO_CARD_CLK_CTL]     @/
 ldrh r0,[r9,#REG_SDIO_CARD_OPTION]      @\
 orr  r0,#0x8000                          @ SDIO_CARD_OPTION set bit15
   bic r0,#0x8000                         @   clear --> want 4bit DATA mode !!!
 strh r0,[r9,#REG_SDIO_CARD_OPTION]      @/
 @---part2b
 ldrh r0,[r9,#REG_SDIO_CARD_OPTION]      @\
 orr  r0,#0x0100                          @ SDIO_CARD_OPTION set bit8
 strh r0,[r9,#REG_SDIO_CARD_OPTION]      @/
 ldrh r0,[r9,#REG_SDIO_CARD_OPTION]      @\
 bic  r0,#0x0100                          @ SDIO_CARD_OPTION clear bit8
 strh r0,[r9,#REG_SDIO_CARD_OPTION]      @/
 @---part3a
 mov  r0,#0x0100                          @\SDIO_CARD_CLK_CTL set bit8
 strh r0,[r9,#REG_SDIO_CARD_CLK_CTL]     @/
 @- - -
 .if try_sdio_data32_mode
      mov  r0,#0x002                           @\want DATA32 mode, step 1
      strh r0,[r9,#REG_SDIO_DATA_CTL]         @/
      ldr  r0,=0x402 @clear fifo, data32 mode @\want DATA32 mode, step 2
      str  r0,[r9,#REG_SDIO_IRQ32]            @/
 .endif
 @- - -

        .if wifi_with_dsi_irq
         ldrh r0,[r9,#REG_SDIO_CARD_IRQ_STAT]             @-read irq stat
         ldrh r0,[r9,#REG_SDIO_CARD_IRQ_DISABLE]          @\
         bic  r0,#1  @Card (RQ                            @ irq disable=off
         strh r0,[r9,#REG_SDIO_CARD_IRQ_DISABLE]          @/
         ldrh r0,[r9,#REG_SDIO_CARD_IRQ_ENABLE]           @\
         orr  r0,#1  @Card (RQ                            @ irq enable=on
         strh r0,[r9,#REG_SDIO_CARD_IRQ_ENABLE]           @/
        .endif

 @- - -
 ldr  r1,=REGBASE_GPIO          @\
 ldrh r0,[r1,#REG_GPIO_WIFI]     @ GPIO_WIFI
 bic  r0,#0x100                   @ <--- NEEDED for DWM-W024 in SDIO mode (else SDIO.func1 fails?!!)
 strh r0,[r1,#REG_GPIO_WIFI]     @/
 @- - -
 mov  r1,#0x30 // mov  r0,13h //  bl set_bptwl_reg_r1_to_r0  @-required for SDIO!
 pop  {pc}
@------------------
sdio_init_opcond_if_needed:
 push {r4-r12,lr}
 mov  r1,#0x0004 // bl sdio_read_func0byte  @bl wrhex8bit @-read interrupt_enable
 ldr  r9,=#REGBASE_SDIO
 ldr  r0,[r9,#REG_SDIO_IRQ_STAT]  @STAT (fail=20C00631h) @\check if passed okay,
 ands r0,#0x400000  @timeout   @\out: r0=need_upload=0    @ if so, skip cmd5 etc.
 beq  skip_opcond_rca      @/                         @/
@- - - if above failed: issue CMD5 ...
 .if verbose_wifi_firmware
   ldr  r1,=txt_init_wifi_op_cond         @\
   bl   wrstr_r1                          @/
 .endif
mov  r8,#1   @need_upload
 mov  r0,#0x00000000  @initially no known voltages        @\
retry_cmd5:  @in: r0=supported voltages               @
 and  r0,#0x00100000  @bit20: 3.2V..3.3V                  @
 bl   sdio_cmd5  @in: r0=param --> reply=10FFFF00h      @ CMD5
@ldr  r9,=REGBASE_SDIO                                  @
 ldr  r0,[r9,#REG_SDIO_IRQ_STAT]  @STAT (fail=20C00631h) @
 tst  r0,#0x400000  @timeout (can occur on W024+CMD5 ?!!) @
 movne r0,#0                                             @
 bne  retry_cmd5 @in: r0                              @
 ldr  r0,[r9,#REG_SDIO_REPLY]    @SDIO_REPLY             @
 tst  r0,#0x80000000 @ready-bit (CMD5 not yet applied)    @
 beq  retry_cmd5 @in: r0                              @/
 mov  r0,#0                                              @\
 bl   sdio_cmd3  @in: r0=param --> reply=00014000h      @ CMD3
@ldr  r9,=REGBASE_SDIO                                  @
 ldr  r0,[r9,#REG_SDIO_REPLY]    @SDIO_REPLY             @
 mov  r0,r0,lsr #16  @isolate RCA                        @/
 mov  r0,r0,lsl #16  @move RCA to MSBs                   @\CMD7
 bl   sdio_cmd7  @in: r0=param --> reply=00001E00h      @/
 mov  r0,#1              @need_upload=1
skip_opcond_rca:
 pop  {r4-r12,pc}     @out: r0=need_upload
@------------------
sdio_init_func0:
 push {lr}
@mov r1,#0x0000 // bl sdio_read_func0byte // bl wrhex8bit @-version_lo
 mov r1,#0x0012 // mov r0,002h // bl sdio_write_func0byte @-cccr power control
 mov r1,#0x0007 // mov r0,082h // bl sdio_write_func0byte @-cccr bus interface (set 4bit mode)
 mov r1,#0x0008 // mov r0,017h // bl sdio_write_func0byte @-cccr card capability (set ...)
 mov r1,#0x0110 // mov r0,80h  // bl sdio_write_func0byte @-fbr1 block size.lsb
 ldr r1,=0x111 // mov r0,00h  // bl sdio_write_func0byte @-fbr1 block size.msb
 mov r1,#0x0002 // mov r0,002h // bl sdio_write_func0byte @-function enable
@@wait_func1ready_lop:                                  @\
 mov r1,#0x0003 // bl sdio_read_func0byte // cmp r0,02h   @ function ready
 bne  wait_func1ready_lop                             @/
 ldr r1,=0x418 // mov r0,0 // bl sdio_write_func1word    @-func1 interrupt_enable
 ldr r1,=0x000040ec // bl sdio_read_intern_word          @\intern chip_id
 ldr r1,=sdio_chip_id                                   @ (02000001h=AR6002)
 str r0,[r1]                                            @ (0D000000h=AR6013)
 .if verbose_wifi_firmware                              @
  ldr r1,=txt_wifi_chip_id                              @
  bl  wrstr_and_hex32bit_and_crlf                       @
 .endif                                                 @/
 pop  {pc}
@------------------
sdio_check_host_interest:
 push {lr}
 ldr  r0,=0x0FFFF   @initial             @\
 ldr  r1,=twlcfg_etc_buf+1E4h @addr     @
 mov  r2,#0x0c       @len                 @
 swi  #swi_crc16 << 16                  @
 ldr  r1,=twlcfg_etc_buf+0x1E0 @addr     @ check CRC16
 ldrh r1,[r1,#0x02]  @[buf+1E2h]=crc      @
 cmp  r1,r0  @crc16                     @
 movne r0,#1   @need_upload @\bad crc    @
 bne   skip_vars_check   @/out: r0    @/
@- - -
@okay, RAM contains addr of host interest (aka vars), so check that...
@- - -
 bl   sdio_vars              @\         @\
 add  r1,#0x58                 @          @ check HOST_RAM[58h]
 bl   sdio_read_intern_word  @/         @  (00000001h=already uploaded)
 cmp  r0,#1    @check if already loaded  @  (or garbage on power-up!!!)
 moveq r0,#0   @need_upload=0 (okay)     @
 movne r0,#1   @need_upload=1 (bad)      @/
skip_vars_check:
 pop  {pc}        @out: r0=need_upload
@------------------
sdio_reset:
 push {lr}
@cmp  xx                   @\skip RESET when... what? (or NEVER skip?)
@bxx  skip_reset         @/
 mov  r1,#0x4000     @RESET_CONTROL       @\
 mov  r0,#0x00000100 @bit8: 1=reset       @ issue reset (bit8: 1=reset, or 0=no change)
 bl   sdio_write_intern_word            @/
 mov  r0,#0x10000                         @\wait (NEED that delay here!)
 swi  swi_wait_by_loop                  @/
 ldr  r1,=#0x40c0    @RESET_CAUSE         @\
 bl   sdio_read_intern_word             @ get reset_cause (00000002h)
 .if verbose_wifi_firmware              @
   ldr  r1,=txt_wifi_reset_cause        @
   bl   wrstr_and_hex32bit_and_crlf     @
 .endif                                 @/
skip_reset:
 pop  {pc}
@------------------
.if with_wifi_firmware_loading
  sdio_load_firmware:
   push {lr}
   .if verbose_wifi_firmware                      @\
     ldr  r1,=txt_loading_wifi_firmware   @\      @
     bl   wrstr_r1                        @/      @
   .endif                                         @
   bl   load_wifi_firmware                @-      @/
   bl   sdio_search_chip_version                  @-
   pop  {pc}
.endif
@------------------
sdio_bmi_init:
 push {lr}
 bl  sdio_bmi_8_get_version             @\
 ldr r1,=sdio_rom_version               @ BMI get version
 str r0,[r1]                            @
 .if verbose_wifi_firmware              @
   ldr r1,=txt_wifi_rom_version         @
   bl  wrstr_and_hex32bit_and_crlf      @
 .endif                                 @/
@- - -
 ldr  r0,=const_00000002h      @src     @\
 bl   sdio_vars  @//add r1,00h @dst     @ BMI write memory
 mov  r2,#4                     @len     @ [500400h] = 00000002h @HOST_RAM[00h]=2
 bl   sdio_bmi_3_write_memory           @/
 ldr  r0,=0x0180C0                       @\
 bl   sdio_bmi_6_read_register          @ BMI read register
 ldr  r1,=sdio_old_local_scratch0       @ 00000000h = [0180C0h] @LOCAL_SCRATCH[0]
 str  r0,[r1]                      @\   @/
 orr  r1,r0,#8                      @/   @\BMI write register
 ldr  r0,=0x0180C0                       @ [0180C0h] = 00000008h @LOCAL_SCRATCH[0]
 bl   sdio_bmi_7_write_register         @/
 ldr  r0,=0x0040C4                       @\
 bl   sdio_bmi_6_read_register          @ BMI read register
 ldr  r1,=sdio_old_wlan_system_sleep    @ 0000001Dh = [0040C4h] @WLAN_SYSTEM_SLEEP
 str  r0,[r1]                      @\   @/   @XXX browser on ar6013 reads 0Dh (not 1Dh)
 orr  r1,r0,#1                      @/   @\BMI write register
 ldr  r0,=0x0040C4                       @ [0040C4h] = 0000001Dh @WLAN_SYSTEM_SLEEP
 bl   sdio_bmi_7_write_register         @/
 mov  r1,#0x00000005                      @\BMI write register
 ldr  r0,=0x004028                       @ [004028h] = 00000005h @-WLAN_CLOCK_CONTROL
 bl   sdio_bmi_7_write_register         @/
 mov  r1,#0x00000000                      @\BMI write register
 ldr  r0,=0x004020                       @ [004020h] = 00000000h @-WLAN_CPU_CLOCK
 bl   sdio_bmi_7_write_register         @/
 pop  {pc}
@------------------
.if with_wifi_firmware_loading
  sdio_bmi_upload_firmware:
   push {lr}
   .if verbose_wifi_firmware
     ldr  r1,=txt_uploading_wifi_stub     @\
     bl   wrstr_r1                        @/
   .endif
   mov  r0,3                              @\
   bl   sdio_get_firmware_part            @ upload stub/data (part d)  @[502400h] = 30h bytes
   bl   sdio_bmi_3_write_memory           @/
   mov  r0,2                              @\
   bl   sdio_get_firmware_part            @ upload stub/code (part c)  @[515000h] = 1F4h+EAh bytes
   bl   sdio_bmi_3_write_memory           @/
   mov  r0,2                              @\
   bl   sdio_get_firmware_part            @ execute [915000h] (aka CODE addr for 515000h)
   add  r0,r1,400000h  @CODE addr         @
   bl   sdio_bmi_4_execute                @/
  @- - -
   .if verbose_wifi_firmware
    ldr  r1,=txt_uploading_wifi_main      @\
    bl   wrstr_r1                         @/
   .endif
  .if with_manual_wifi_decompress
      mov  r0,0                              @\
      bl   sdio_get_firmware_part            @ lz_start (502400h) (part a)
      mov  r0,r1  @dst  @502400h             @
      bl   sdio_bmi_D_lz_start               @/
      mov  r0,0                              @\
            mov r2,2  @XXX dummy len
      bl   sdio_get_firmware_part            @ lz_data, send (1F8h+8h)*N+58h bytes
      bl   sdio_bmi_E_lz_data                @/

   mov  r0,0                              @\
   bl   sdio_get_firmware_part            @ decompress/upload main (part a)
   bl   manual_lz_decode
       mov  r0,0                              @\lz_start (0) blah
       bl   sdio_bmi_D_lz_start               @/

  .else
         @bl show_timer
   mov  r0,0                              @\
   bl   sdio_get_firmware_part            @ lz_start (502400h) (part a)
   mov  r0,r1  @dst  @502400h             @
   bl   sdio_bmi_D_lz_start               @/
   bl   sdio_bmi_wait_count4              @-wait for COUNT[4] nonzero (extra after above FIRST call to bmi_D ?)
   mov  r0,0                              @\
   bl   sdio_get_firmware_part            @ lz_data, send (1F8h+8h)*N+58h bytes
   bl   sdio_bmi_E_lz_data                @/
  @mov  r0,0                              @\lz_start (0) blah
  @bl   sdio_bmi_D_lz_start               @/
         @bl show_timer
          @W015: 354ms
          @W024: 32ms
  .endif
   mov  r0,3                              @\
   bl   sdio_get_firmware_part            @ upload stub/data (part d)  @[502400h] = 30h bytes  @again ???
   bl   sdio_bmi_3_write_memory           @/
   mov  r0,1                              @\
   bl   sdio_get_firmware_part            @ upload database  (part b)  @[52D944h] = 1F4h+C8h bytes
   ldr  r3,=sdio_database_addr @\memorize @
   str  r1,[r3]                @/         @
   bl   sdio_bmi_3_write_memory           @/
   bl   sdio_vars // add r1,18h @=500418h @\
   ldr  r0,=sdio_database_addr  @src      @ [500418h] = [src]=0052D944h
  @@@BUG:  ldr  r0,[r0]  @<--- ARGH, this indirection was wrong... and took 3-4 day to figure out why it didn't work
   mov  r2,4                    @len      @
   bl   sdio_bmi_3_write_memory           @/
   pop  {pc}
.endif
@------------------
sdio_bmi_finish:
 push {lr}
 ldr  r1,=sdio_old_wlan_system_sleep    @\
 ldr  r1,[r1]                           @ [0040C4h] = 0000001Ch @-WLAN_SYSTEM_SLEEP
 bic  r1,#1                              @    @XXX browser on ar6013 has old=0Dh (not 1Dh)
 ldr  r0,=0x0040C4                       @    @    and so, writes 0Ch (not 1Ch)
 bl   sdio_bmi_7_write_register         @/
 ldr  r1,=sdio_old_local_scratch0       @\
 ldr  r1,[r1]                           @ [0180C0h] = 00000000h @-LOCAL_SCRATCH[0]
 ldr  r0,=0x0180C0                       @
 bl   sdio_bmi_7_write_register         @/
 ldr  r0,=const_00000080h               @\
 bl   sdio_vars // add r1,6Ch           @ [50046Ch] = 00000080h @-HOST_RAM[6Ch] hi_mbox_io_block_sz
 mov  r2,#4                              @
 bl   sdio_bmi_3_write_memory           @/
 ldr  r0,=const_00000063h               @\
 bl   sdio_vars // add r1,74h           @ [500474h] = 00000063h @-HOST_RAM[74h] hi_mbox_isr_yield_limit
 mov  r2,#4                              @
 bl   sdio_bmi_3_write_memory           @/
 .if verbose_wifi_firmware
   ldr  r1,=txt_starting_wifi_main      @\
   bl   wrstr_r1                        @/
 .endif
 bl   sdio_bmi_1_done                   @-done, launch firmware !!!!!!
wait_for_firmware_init_done:          @\
 bl   sdio_vars // add r1,58h   @src    @ wait until launched/ready
 bl   sdio_read_intern_word             @ 0000000xh = [500458h]
 cmp  r0,#1                              @ (wait until changing from 0 to 1)
 bne  wait_for_firmware_init_done     @/
 pop  {pc}
@------------------
sdio_get_eeprom_stuff:
 push {lr}
 bl   sdio_vars // add r1,54h           @\
 bl   sdio_read_intern_word             @ get EEPROM address @HOST_RAM[54h]
 ldr  r1,=sdio_eeprom_addr              @
 str  r0,[r1]                           @/
 ldr  r1,=sdio_eeprom_addr              @\
 ldr  r1,[r1]                           @
@add  r1,00h                            @ get EEPROM[000h] (300h, size maybe?)
 bl   sdio_read_intern_word             @
@str  XXX ?                             @/
 ldr  r1,=sdio_eeprom_addr              @\
 ldr  r1,[r1]                           @
 add  r1,#0x10                            @ get EEPROM[010h] (version or so)
 bl   sdio_read_intern_word             @
@str  XXX ?                             @
 .if verbose_wifi_firmware              @
   ldr r1,=txt_wifi_eeprom_version      @
   bl  wrstr_and_hex32bit_and_crlf      @
 .endif                                 @/
 pop  {pc}
@------------------
sdio_whatever_handshake:
 push {lr}
 .if verbose_wifi_firmware
   ldr  r1,=txt_sending_wifi_handshake  @\
   bl   wrstr_r1                        @/
 .endif
 bl   sdio_recv_mbox_block              @\                    @HTC_MSG_READY_ID
 ldr  r0,=sdio_mbox_1st_handshake       @ MBOX 1st handshake  @WMI_CONTROL
 bl   sdio_send_mbox_block              @/
 bl   sdio_recv_mbox_block              @\
 ldr  r0,=sdio_mbox_2nd_handshake       @ MBOX 2nd handshake  @WMI_DATA_BE best effort
 bl   sdio_send_mbox_block              @/
 bl   sdio_recv_mbox_block              @\
 ldr  r0,=sdio_mbox_3rd_handshake       @ MBOX 3rd handshake  @WMI_DATA_BK background
 bl   sdio_send_mbox_block              @/
 bl   sdio_recv_mbox_block              @\
 ldr  r0,=sdio_mbox_4th_handshake       @ MBOX 4th handshake  @WMI_DATA_VI video
 bl   sdio_send_mbox_block              @/
 bl   sdio_recv_mbox_block              @\
 ldr  r0,=sdio_mbox_5th_handshake       @ MBOX 5th handshake  @WMI_DATA_CO voice
 bl   sdio_send_mbox_block              @/
 bl   sdio_recv_mbox_block              @\
 ldr  r0,=sdio_mbox_6th_handshake       @ MBOX 6th handshake  @?
 bl   sdio_send_mbox_block              @/
 @- - -
 ldr  r1,=0x418   @INT_STATUS_ENABLE     @\
 ldr  r0,=0x010300D1                     @ enable IRQs
         mov r0,#1  @bit0=MBOX0notemptyIRQ            @XXXXXXXX
 bl   sdio_write_func1word              @/
 mov  r1,#0x0004   @CCCR irq_enable       @\
 mov  r0,#0x003    @master+func1          @ CCCR interrupt_enable (enable master & func1 interrupts)
 bl   sdio_write_func0byte              @/
wait_func_irq_lop:                    @\
 mov  r1,#0x0005   @CCCR irq_flags        @ wait for
 bl   sdio_read_func0byte               @ CCCR interrupt_flags   @XXX wait for CardIRQ to occur via IF2 and [4A36h]
 tst  r0,#0x02                            @
 beq  wait_func_irq_lop               @/
 .if with_wifi_firmware_loading
   @these are normally processed/flushed by firmware uploader
   @however, outside of firmware, they can be processed later on
   @(ie. then processed by the event handler in wifiboot)
   bl   sdio_recv_mbox_block            @-RECV (1001h: WMI_READY_EVENT)
   bl   sdio_recv_mbox_block            @-RECV (1006h: WMI_REGDOMAIN_EVENT) @aka REG_DOMAIN
   bl   sdio_check_mbox_state           @-check (but get nothing)
 .endif
 bl   sdio_vars                         @\
@add  r1,0x00                            @ get 00507470h = [500400h]  @HOST_RAM[00h]
 bl   sdio_read_intern_word      @\     @/
 mov  r1,r0  @addr=507470h       @/     @\
 mov  r0,#2   @data=2                    @ set [507470h] = 00000002h  @RAM
 bl   sdio_write_intern_word            @/
 pop  {pc}
@------------------
sdio_check_mbox_state:
 push {lr}
 ldr  r0,=sdio_xfer_buf @dst (temp buf) @\
 ldr  r1,=0x10000400     @src (state)    @
 mov  r2,#0x0c            @len            @
 bl   sdio_cmd53_read                   @/  @in: r0=dst(mem), r1=src(io), r2=len

@.comment
@   ldr  r1,=sdio_xfer_buf
@   mov  r2,#0x0c
@  lop:
@   tst  r2,#3
@   bleq wrspc
@   ldrb r0,[r1],#1
@   bl   wrhex8bit
@   subs r2,#1
@   bne  lop
@   bl   wrcrlf
@.comment

 pop  {pc}
@------------------
sdio_recv_mbox_block:
 push {r4,lr}
@mov  r4,#0x100  @timeout <-- TOO small for W024 \
 mov  r4,#0x1000 @timeout <-- ok for W024 @\
wait_lop:                             @
 subs r4,#1     @timeout                 @
 beq  timeout                         @
 bl   sdio_check_mbox_state             @
 ldr  r0,=sdio_xfer_buf                 @ wait if mbox empty
 ldrb r0,[r0]                           @
 tst  r0,#1     @test mbox0 not empty    @
 beq  wait_lop                        @/
 ldr  r0,=sdio_xfer_buf @dst (temp buf) @\
 ldr  r1,=0x18001000-0x80 @src (mbox/blk) @ receive mbox block
 mov  r2,#1              @len (1 blk)    @
 bl   sdio_cmd53_read                   @/  @in: r0=dst(mem), r1=src(io), r2=len

.if verbose_wifi_firmware @AND 01
     ldr  r1,=sdio_xfer_buf
     mov  r2,#0x10
    lop:
     ldrb r0,[r1],#1
     bl   wrhex8bit
     subs r2,#1
     bne  lop
    @@@ bl   wrcrlf
.endif

done1:
 pop  {r4,pc}
@---
timeout:
 .if verbose_wifi_firmware
   bl   wrdot
 .endif
 b    done1
@------------------
sdio_send_mbox_block:   @in: r0=src
 push {r4,lr}
 ldrh r4,[r0,#2]         @len            @\get len
 add  r4,#6              @len+6          @/
 ldr  r1,=sdio_xfer_buf @dst            @\
 mov  r2,r4             @len            @
 cmp   r0,r1              @\omit/skip   @
 addeq r0,r2 @skip.src    @ copy if     @ copy to xfer buf
 addeq r1,r2 @skip.dst    @ src=len     @
 moveq r2,#0  @expire.len  @/(for data)  @
 blne memcopy_bytewise                  @/

@XXX try to make below zerofill optional, via with_gimmicks or so
 mov  r0,r1  @dst (increased above)     @\zeropad xfer buf to 80h-byte boundary
 rsb  r1,r4,#0          @0-len           @ (not needed, could also send garbage)
 and  r1,#0x007f         @0..7Fh          @
 bl   zerofill_bytewise                 @/

 add  r4,#0x7f                            @\round-up len
 bic  r4,#0x7f                            @/
 ldr  r0,=sdio_xfer_buf @src (temp buf) @\
 ldr  r1,=0x18001000     @dst (mbox/blk) @ send mbox block
 sub  r1,r4  @dst-len                   @
 mov  r2,r4,lsr #7       @len/80h (blk's)@
 bl   sdio_cmd53_write                  @/
 pop  {r4,pc}
@------------------
sdio_read_intern_word:
 push {r1-r12,lr}
 mov  r8,r1                                             @-addr
 ldr  r0,=sdio_xfer_buf @src (temp buf) @\              @\
 ldr  r1,=0x1000047c+1   @dst + func1    @               @
 mov  r2,#3              @len            @ (bit8-31)     @
 mov  r3,r8,lsr #8       @upper 24bit    @               @ send WINDOW_READ_ADDR
 str  r3,[r0]           @[src]=data     @               @
 bl   sdio_cmd53_write  @--> write      @/              @
 ldr  r0,=sdio_xfer_buf @src (temp buf) @\              @
 ldr  r1,=0x1000047c+0   @dst + func1    @               @
 mov  r2,#1              @len            @ (bit0-7)      @
 and  r3,r8,#0x0ff        @lower 8bit     @ (LSB last!)   @
 str  r3,[r0]           @[src]=data     @               @
 bl   sdio_cmd53_write  @--> write      @/              @/
 ldr  r1,=0x0474                                         @\read WINDOW_DATA
 bl   sdio_read_func1word  @in: r1=addr, out: r0=data   @/
 pop  {r1-r12,pc}
@---
sdio_write_intern_word:
 push {r0-r12,lr}
 mov  r8,r1             @-addr
 ldr  r1,=0x0474         @dst                            @\
@mov  r0,r0             @data                           @ WINDOW_DATA (32bit)
 bl   sdio_write_func1word                              @/
 ldr  r0,=sdio_xfer_buf @src (temp buf) @\              @\
 ldr  r1,=0x10000478+1   @dst + func1    @               @
 mov  r2,#3              @len            @ (bit8-31)     @
 mov  r3,r8,lsr #8       @upper 24bit    @               @
 str  r3,[r0]           @[src]=data     @               @ WINDOW_WRITE_ADDR
 bl   sdio_cmd53_write  @--> write      @/              @
 ldr  r0,=sdio_xfer_buf @src (temp buf) @\              @
 ldr  r1,=0x10000478+0   @dst + func1    @               @
 mov  r2,#1              @len            @ (bit0-7)      @
 and  r3,r8,#0x0ff        @lower 8bit     @ (LSB last!)   @
 str  r3,[r0]           @[src]=data     @               @
 bl   sdio_cmd53_write  @--> write      @/              @/
 pop  {r0-r12,pc}
@------------------
.if with_wifi_firmware_loading
  sdio_search_chip_version:
   push {r4,lr}
   ldr  r4,=twlcfg_etc_buf+1E0h                   @\wifi board version
   ldrb r4,[r4]                                   @/(from WifiFlash[1FDh]
   ldr  r3,=sdio_firmware_buf                     @-
   ldrb r2,[r3,#0x0a2]      @num.parts              @\
   add  r1,r3,#0x0a4        @parts.list             @
  @@part_lop:                                     @
   ldr  r0,[r1,#0x08]  @ID  @parts.list[8]          @ search part with
   cmp  r0,r4        @ID  @WifiFlash[1FDh]        @ matching ID
   beq  @@found                                   @
   add  r1,#0x20            @parts.list             @
   subs r2,#1              @num.parts              @
   bne  @@part_lop                                @/
   ldr  r1,=txt_error_wifi_firmware_not_found     @\
   b    error_wrstr_r1                            @/
  @---
  @@found:
   ldr  r1,[r1,#0x00] @addr @parts.list[0]  @\
   add  r1,r3       @addr @firmware_buf   @
   ldr  r3,=sdio_firmware_part_base       @
   str  r1,[r3]                           @/

      @XXX get/check chip_id
      @XXX get/check rom_version

   ldrb r2,[r1,01h] @num.chip.id's        @\
   ldrh r3,[r1,02h] @offs.to.chip.id's    @
   add  r3,r3,r2,lsl 3 @offs+num*8        @
   add  r3,4           @offs+num*8+4      @ get RAM vars/base/size
   add  r0,r1,r3  @part+offs+num*8+4 @src @
   ldr  r1,=twlcfg_etc_buf+1E4h      @dst @
   mov  r2,0ch                       @len @
   bl   memcopy_bytewise                  @/
   ldr  r0,=0FFFFh   @initial             @\
   ldr  r1,=twlcfg_etc_buf+1E4h @addr     @
   mov  r2,0ch       @len                 @
   swi  swi_crc16 shl 16                  @
   ldr  r1,=twlcfg_etc_buf+1E0h @addr     @ adjust CRC16
   strh r0,[r1,02h]  @[20005E2h]=crc      @ and some unknown/zero bytes
   mov  r0,0                              @
   strb r0,[r1,01h]  @[20005E1h]=0        @
   str  r0,[r1,10h]  @[20005F0h]=0        @
   str  r0,[r1,14h]  @[20005F4h]=0        @
   str  r0,[r1,18h]  @[20005F8h]=0        @
   str  r0,[r1,1ch]  @[20005FCh]=0        @/
   pop  {r4,pc}
.endif
@------------------
sdio_vars:      @aka host_interest_area
 ldr  r1,=twlcfg_etc_buf+0x1E4
 ldr  r1,[r1]   @get addr of host_interest_area
 bx   lr
@------------------
sdio_get_firmware_part:  @in: r0=0..3
 ldr  r3,=sdio_firmware_part_base
 ldr  r3,[r3]   @eg. "sdio_firmware_buf+100h" for part1
 add  r1,r3,#4         @subpart 0
 add  r0,r1,r0,lsl #4  @subpart 0..3
@ldr  xx,[r0,#0x08]  @flags
 ldr  r2,[r0,#0x04]  @len
 ldr  r1,[r0,#0x0c]  @dst addr (in atheros memory)
 ldr  r0,[r0,#0x00]  @src offset (in file part)
 add  r0,r3        @src offset (in memory)
 bx   lr
@------------------
sdio_read_mbox0word:  @out: r0=data
 ldr  r1,=0x0ffc
@- - - - - --------
sdio_read_func1word:  @in: r1=addr, out: r0=data
 push {lr}
 ldr  r0,=sdio_xfer_buf     @dst (temp buf)
 orr  r1,#1 << 28  @func1   @src + func1
 mov  r2,#4                  @len
 bl   sdio_cmd53_read    @in: r0=sdio_xfer_buf=dst, r1=src, r2=len
 ldr  r0,=sdio_xfer_buf     @dst (temp buf)
 ldr  r0,[r0]               @data[dst]
 pop  {pc}
@------------------
sdio_read_func1byte:  @in: r1=addr, out: r0=data
 push {lr}
 ldr  r0,=sdio_xfer_buf     @dst (temp buf)
 orr  r1,#1 << 28  @func1   @src + func1
 mov  r2,#1                  @len
 bl   sdio_cmd53_read    @in: r0=sdio_xfer_buf=dst, r1=src, r2=len
 ldr  r0,=sdio_xfer_buf     @dst (temp buf)
 ldrb r0,[r0]               @data[dst]
 pop  {pc}
@------------------
sdio_write_mbox0word:  @in: r0=data
 ldr  r1,=0x0ffc
@- - - - - --------
sdio_write_func1word:  @in: r0=data, r1=addr
 mov  r3,r0                 @data
 ldr  r0,=sdio_xfer_buf     @src (temp buf)
 orr  r1,#1 << 28  @func1   @dst + func1
 mov  r2,#4                  @len
 str  r3,[r0]               @[src]=data
 b    sdio_cmd53_write   @in: r0=sdio_xfer_buf=src, r1=dst, r2=len
@------------------
sdio_bmi_wait_count4:
 push {lr}
wait_count4_lop:                      @\
 mov  r1,#0x450                           @
 bl   sdio_read_func1byte               @ wait for COUNT[4] nonzero
 cmp  r0,#0x00                            @
 beq  wait_count4_lop                 @/
 pop  {pc}
@------------------
sdio_bmi_1_done:
 push {lr}
 bl   sdio_bmi_wait_count4              @-wait for COUNT[4] nonzero
 mov  r0,#0x01                            @\send BMI command done, launch firmware
 bl   sdio_write_mbox0word              @/
 pop  {pc}
@------------------
sdio_bmi_2_read_memory:  @in: r0=src(sdio), r1=dst(arm), r2=len
 push {r4-r7,lr}
 mov  r4,r0        @src
 mov  r5,r1        @dst
 mov  r6,r2        @remain
fragment_lop:
 mov   r7,r6  @len=remain               @\
 cmp   r7,#0x80 @max mbox size (only 80h?)@ compute fragment size
 movhi r7,#0x80                           @/
 bl   sdio_bmi_wait_count4              @-wait for COUNT[4] nonzero
 ldr  r0,=sdio_xfer_buf @src (temp buf) @\
 ldr  r1,=0x10001000-0x0c @dst (mbox)     @
 mov  r2,#0x0c            @len            @
 mov  r3,#0x02  @cmd                      @ send cmd+addr+len
 str  r3,[r0,#0x00]  @cmd                 @
 str  r4,[r0,#0x04]  @addr                @
 str  r7,[r0,#0x08]  @len                 @
 bl   sdio_cmd53_write                  @/
 mov  r2,r7             @len            @\
 bl   sdio_cmd53_read_mbox_to_xfer_buf  @/
 ldr  r0,=sdio_xfer_buf         @src    @\
 mov  r0,r5                     @dst    @ copy data(len) from xfer buf
 mov  r2,r7                     @len    @
 bl   memcopy_bytewise                  @/
 add  r4,r7  @src                       @\
 add  r5,r7  @dst                       @ lop next fragment
 subs r6,r7  @remain                    @
 bne  fragment_lop                    @/
 pop  {r4-r7,pc}
@------------------
sdio_cmd53_read_mbox_to_xfer_buf:  @in: r2=len
 ldr  r0,=sdio_xfer_buf @dst (temp buf) @\
 ldr  r1,=0x10001000     @src (mbox)     @
 sub  r1,r2             @mbox-len       @
@mov  r2,r2             @len            @
 b    sdio_cmd53_read                   @/
@------------------
sdio_bmi_3_write_memory:  @in: r0=src(arm), r1=dst(sdio), r2=len
 push {r4-r7,lr}
 mov  r4,r0        @src
 mov  r5,r1        @dst
 mov  r6,r2        @remain
@@fragment_lop:
 mov   r7,r6  @len=remain               @\
 cmp   r7,#0x200h-0x0c @max mbox size       @ compute fragment size
 movhi r7,#0x200-0x0c                      @/
 mov  r0,r4                     @src    @\
 ldr  r1,=sdio_xfer_buf+0ch     @dst    @ copy data(len) to xfer buf
 mov  r2,r7                     @len    @
 bl   memcopy_bytewise                  @/
 bl   sdio_bmi_wait_count4              @-wait for COUNT[4] nonzero
 ldr  r0,=sdio_xfer_buf @src (temp buf) @\
 ldr  r1,=0x10001000     @dst (mbox)     @
 add  r2,r7,#0x0c         @len            @
 sub  r1,r2             @mbox-len       @
 mov  r3,#0x03  @cmd                      @ send cmd+addr+len+data(len)
 str  r3,[r0,#0x00]  @cmd                 @
 str  r5,[r0,#0x04]  @addr                @
 str  r7,[r0,#0x08]  @len                 @
 bl   sdio_cmd53_write                  @/
 add  r4,r7  @src                       @\
 add  r5,r7  @dst                       @ lop next fragment
 subs r6,r7  @remain                    @
 bne  fragment_lop                    @/
 pop  {r4-r7,pc}
@------------------
.if with_wifi_firmware_loading
  sdio_bmi_4_execute:
   push {r4-r5,lr}
   mov  r4,r0        @entrypoint
   mov  r5,r1        @argument
   bl   sdio_bmi_wait_count4              @-wait for COUNT[4] nonzero
   ldr  r0,=sdio_xfer_buf @src (temp buf) @\
   ldr  r1,=0x10001000-0x0c @dst (mbox)     @
   mov  r2,#0x0c            @len            @
   mov  r3,#0x04  @cmd                      @ send cmd+entry+arg
   str  r3,[r0,#0x00]  @cmd                 @
   str  r4,[r0,#0x04]  @entrypoint          @
   str  r5,[r0,#0x08]  @argument            @
   bl   sdio_cmd53_write                  @/
   bl   sdio_read_mbox0word               @-recv return value
   pop  {r4-r5,pc}
.endif
@------------------
sdio_bmi_6_read_register:
 push {r4-r5,lr}
 mov  r4,r0        @addr
 bl   sdio_bmi_wait_count4              @-wait for COUNT[4] nonzero
 ldr  r0,=sdio_xfer_buf @src (temp buf) @\
 ldr  r1,=0x10001000-0x08 @dst (mbox)     @
 mov  r2,#0x08            @len            @
 mov  r3,#0x06  @cmd                      @ send cmd+addr
 str  r3,[r0,#0x00]  @cmd                 @
 str  r4,[r0,#0x04]  @addr                @
 bl   sdio_cmd53_write                  @/
 bl   sdio_read_mbox0word               @-recv data
 pop  {r4-r5,pc}
@------------------
sdio_bmi_7_write_register:
 push {r4-r5,lr}
 mov  r4,r0        @addr
 mov  r5,r1        @data
 bl   sdio_bmi_wait_count4              @-wait for COUNT[4] nonzero
 ldr  r0,=sdio_xfer_buf @src (temp buf) @\
 ldr  r1,=0x10001000-0x0c @dst (mbox)     @
 mov  r2,#0x0c            @len            @
 mov  r3,#0x07  @cmd                      @ send cmd+addr+data
 str  r3,[r0,#0x00]  @cmd                 @
 str  r4,[r0,#0x04]  @addr                @
 str  r5,[r0,#0x08]  @data                @
 bl   sdio_cmd53_write                  @/
 pop  {r4-r5,pc}
@------------------
sdio_bmi_8_get_version:
 push {r4,lr}
 bl   sdio_bmi_wait_count4              @-wait for COUNT[4] nonzero
 mov  r0,#0x08                            @\send BMI command get_version
 bl   sdio_write_mbox0word              @/
 bl   sdio_read_mbox0word               @\get rom version
 cmp  r0,#0x0ffffffff                     @ (or FFFFFFFFh=extended)
 bne  this_rom_version  @r0=ver       @/
@- - -
 bl   sdio_read_mbox0word               @\get len of extended data
 sub  r2,r0,#4   @len=total-4            @ and read remaining extended data
 bl   sdio_cmd53_read_mbox_to_xfer_buf  @/
 ldr  r0,=sdio_xfer_buf                 @\get rom version
 ldr  r0,[r0]   @1st word = rom version @/
this_rom_version:
 pop  {r4,pc}
@------------------
.if with_wifi_firmware_loading
  sdio_bmi_D_lz_start:
   push {r4,lr}
   mov  r4,r0        @dst
   bl   sdio_bmi_wait_count4              @-wait for COUNT[4] nonzero
   ldr  r0,=sdio_xfer_buf @src (temp buf) @\
   ldr  r1,=0x10001000-0x08 @dst (mbox)     @
   mov  r2,#0x08            @len            @
   mov  r3,#0x0d  @cmd                      @ send cmd+addr
   str  r3,[r0,#0x00]  @cmd                 @
   str  r4,[r0,#0x04]  @addr                @
   bl   sdio_cmd53_write                  @/
   pop  {r4,pc}
.endif
@------------------
.if with_wifi_firmware_loading
  sdio_bmi_E_lz_data:  @in: r0=src(arm), r2=len
   push r4-r7,lr
   mov  r4,r0        @src
   mov  r6,r2        @remain
  @@fragment_lop:
   mov   r7,r6  @len=remain               @\
   cmp   r7,200h-08h @max mbox size       @ compute fragment size
   movhi r7,200h-08h                      @/
   mov  r0,r4                     @src    @\
   ldr  r1,=sdio_xfer_buf+08h     @dst    @ copy data(len) to xfer buf
   mov  r2,r7                     @len    @
   bl   memcopy_bytewise                  @/
   bl   sdio_bmi_wait_count4              @-wait for COUNT[4] nonzero
   ldr  r0,=sdio_xfer_buf @src (temp buf) @\
   ldr  r1,=10001000h     @dst (mbox)     @
   add  r2,r7,08h         @len            @
   sub  r1,r2             @mbox-len       @
   mov  r3,0eh  @cmd                      @ send cmd+addr+len+data(len)
   str  r3,[r0,00h]  @cmd                 @
   str  r7,[r0,04h]  @len                 @
   bl   sdio_cmd53_write                  @/
   add  r4,r7  @src                       @\
   subs r6,r7  @remain                    @ lop next fragment
   bne  @@fragment_lop                    @/
   pop  r4-r7,pc
.endif
@------------------
.if with_wifi_firmware_loading
 .if with_manual_wifi_decompress
  manual_lz_decode:
   push r4-r9,lr
         @bl show_timer
   mov  r7,r1     @dst    @-memorize dst.start
   mov  r1,sdio_lz_buf @dst
   add  r8,r0,r2  @end    @-calc end (src+len)
   ldrb r9,[r0],1 @src    @-fetch tag
  @@decompress_lop:       @\
   ldrb r3,[r0],1 @src    @
   cmp  r3,r9     @tag    @
   beq  @@tag             @ decompress
  @@store_r3:             @
   strb r3,[r1],1 @dst    @
  @@decompress_next:      @
   cmp  r0,r8     @end    @
   bne  @@decompress_lop  @/
  @- - -
         @bl show_timer
          @W015: 266ms
          @W024: 36ms
   mov  r0,sdio_lz_buf @src       @\
   mov  r2,r1          @dst.end   @
   sub  r2,r0          @len       @ upload
   mov  r1,r7          @dst       @
   bl   sdio_bmi_3_write_memory   @/
         @bl show_timer
          @W015: 528ms ... for 26E75h bytes?
          @W024: 52ms  ... for 533Bh bytes?
   pop  r4-r9,pc
  @---
  @@tag:
   bl   @@fetch_r3        @\
   movs r2,r3     @len    @ fetch len
   mov  r3,r9     @tag    @ if zero: store tag
   beq  @@store_r3        @/
   bl   @@fetch_r3        @\fetch disp
   sub  r4,r1,r3  @src'   @/and calc src'=dst-disp
  @@copy_lop:             @\
   ldrb r3,[r4],1 @src'   @
   strb r3,[r1],1 @dst    @ copy from dst-disp to dst
   subs r2,1      @len    @
   bne  @@copy_lop        @/
   b    @@decompress_next
  @---
  @@fetch_r3:
   mov  r3,0    @initial sum
  @@fetch_r3_lop:
   ldrb r4,[r0],1
   tst  r4,80h   @0=endflag
   bic  r4,80h
   orr  r3,r4,r3,lsl 7
   bne  @@fetch_r3_lop
   bx   lr
 .endif
.endif
@------------------
.if verbose_wifi_firmware
wrstr_and_hex32bit_and_crlf:
 push lr
 bl   wrstr_r1
 bl   wrhex32bit
 mov  r0,attr_orange
 call wrchr_r0
 bl   wrcrlf
 pop  pc
@---
txt_wifi_chip_id        db 'CHIP ID:'        ,attr_white,0
txt_wifi_reset_cause    db 'RESET CAUSE:'    ,attr_white,0
txt_wifi_eeprom_version db 'EEPROM VERSION:' ,attr_white,0
txt_wifi_rom_version    db 'ROM VERSION:'    ,attr_white,0
.align 4
.endif @verbose_wifi_firmware
@------------------
sdio_read_func0byte:   @in: r1=addr, out: r0=data
 push {r1-r12,lr}
 bl   sdio_read_register_r1        @in: r1=addr(bit0-16)+func(bit28-30)
 pop  {r1-r12,pc}
@---
sdio_write_func0byte:  @in: r0=data, r1=addr
 push {r0-r12,lr}
 bl   sdio_write_r0_to_register_r1 @in: r0=data(8bit), r1=addr(bit0-16)+func(bit28-30)
 pop  {r0-r12,pc}
@------------------
sdio_read_register_r1:   @in: r1=addr(bit0-16)+func(bit28-30)
 push {r1-r2,lr}
@@@@@param equ (04000100h + (@@func shl 28))
 @---part3b
 mov  r0,r1,lsl #9     @data(NONE), addr(bit9-25)        @\
 and  r2,r1,(#7 shl #28)@func(bit28-30)                   @ param
 orr  r0,r0,r2        @func(bit28-30)                   @/
 ldr  r1,=0x0434    @single-byte CMD52                   @-cmd
 b    sdio_access_register_core_inj
@------------------
sdio_write_r0_to_register_r1: @in: r0=data(8bit), r1=addr(bit0-16)+func(bit28-30)
 push {r1-r2,lr}
 @---part3b
 orr  r0,r0,r1,lsl #9  @data(bit0-7), addr(bit9-25)      @\
 and  r2,r1,(#7 shl #28)@func(bit28-30)                   @ param
 orr  r0,r0,r2        @func(bit28-30)                   @
 orr  r0,#1 << 31     @writeflag(bit31)                 @/
 ldr  r1,=0x0434    @single-byte CMD52                   @-cmd
 b    sdio_access_register_core_inj
@------------------
sdio_cmd3:  @in: r0=param(32bit)    @GET_RELATIVE_ADDR
 push {r1-r2,lr}
 ldr  r1,=0x0403    @single-byte CMD3                    @-cmd
 b    sdio_access_register_core_inj
@------------------
sdio_cmd5:  @in: r0=param(32bit)    @SDIO_SEND_OP_COND
 push {r1-r2,lr}
 ldr  r1,=0x0705    @single-byte CMD5                    @-cmd
 b    sdio_access_register_core_inj
@------------------
sdio_cmd7:  @in: r0=param(32bit)    @SELECT_DESELECT_CARD
 push {r1-r2,lr}
 ldr  r1,=0x0507    @single-byte CMD7                    @-cmd
 b    sdio_access_register_core_inj
@------------------
sdio_access_register_core_inj:  @in: r0=param, r1=cmd, out: r0=data (if reading)
 ldr  r9,=REGBASE_SDIO
 str  r0,[r9,#REG_SDIO_CMD_PARAM]        @-PARAM
 ldr  r0,[r9,#REG_SDIO_IRQ_STAT]         @\SDIO_IRQ_STAT
 bic  r0,#0x83000000                      @ clear bit31,25,24 (error, txrq, rxrdy)
 bic  r0,#0x007f0000                      @ clear bit22..16   (error)
 bic  r0,#0x00000005                      @ clear bit2,0      (dataend,cmdrespend)
 str  r0,[r9,#REG_SDIO_IRQ_STAT]         @/
 ldrh r0,[r9,#REG_SDIO_STOP_INTERNAL]    @\
 bic  r0,#1                              @ SDIO_STOP_INTERNAL clear bit0
 strh r0,[r9,#REG_SDIO_STOP_INTERNAL]    @/
 strh r1,[r9,#REG_SDIO_CMD]              @-SDIO_CMD
 @- - -
busy_lop:
 ldr  r0,[r9,#REG_SDIO_IRQ_STAT]         @\
 tst  r0,#0x7f0000 @ERR                   @
 bne  error                           @ wait busy
 tst  r0,#1       @CMDRESPEND            @
 beq  busy_lop                        @/
 ldrh r0,[r9,#REG_SDIO_IRQ_STAT]         @\
 tst  r0,#1 << 22    @bit22, CMDTIMEOUT @ is that really needed here?
 bne  error_hw_timeout                @/
 ldr  r0,[r9,#REG_SDIO_REPLY]    @\SDIO_REPLY (00001000h, ie. state = "dis"@ if it's "CSR")
 and  r0,#0x00ff                  @/
done2:
 pop  {r1-r2,pc}
@---
error:
error_hw_timeout:
 .if verbose_wifi_firmware
   bl   wrdot
 .endif
 mov  r0,#-1
 b    done2
@------------------
sdio_cmd53_read:   @in: r0=dst(mem), r1=src(io), r2=len
 push {r4-r12,lr}
 mov  r11,r0   @dst
 mov  r12,r2   @len

 mov  r4,r1,lsl #9     @move addr to bit9-25     @\
 and  r1,(#7 shl #28)+(#1 shl #27) @isolate func(bit28-30)+blockmode(bit27)
 orr  r4,r4,r1        @merge addr+func          @
 orr  r4,#1 << 26     @incrementing.addr(bit26) @
@orr  r4,#0 << #31     @writeflag(bit31)=0       @
 bic  r3,r2,#0x200      @crop len=200h to 000h    @
 orr  r4,r3           @merge len (bit0-8)       @/

 b    sdio_cmd53_access_inj
@------------------
sdio_cmd53_write:   @in: r0=src(mem), r1=dst(io), r2=len
 push {r4-r12,lr}
 mov  r11,r0   @src
 mov  r12,r2   @len

 mov  r4,r1,lsl #9     @move addr to bit9-25     @\
 and  r1,(#7 shl #28)+(#1 shl #27) @isolate func(bit28-30)+blockmode(bit27)
 orr  r4,r4,r1        @merge addr+func          @
 orr  r4,#1 << 26     @incrementing.addr(bit26) @
 orr  r4,#1 << 31     @writeflag(bit31)         @
 bic  r3,r2,#0x200      @crop len=200h to 000h    @
 orr  r4,r3           @merge len (bit0-8)       @/

 b    sdio_cmd53_access_inj
@------------------
sdio_cmd53_access_inj:
 ldr  r9,=#REGBASE_SDIO
 str  r4,[r9,#REG_SDIO_CMD_PARAM]        @-param
 ldr  r0,[r9,#REG_SDIO_IRQ_STAT]         @\SDIO_IRQ_STAT
 bic  r0,#0x83000000                      @ clear bit31,25,24 (error, txrq, rxrdy)
 bic  r0,#0x007f0000                      @ clear bit22..16   (error)
 bic  r0,#0x00000005                      @ clear bit2,0      (dataend,cmdrespend)
 str  r0,[r9,#REG_SDIO_IRQ_STAT]         @/
 ldrh r0,[r9,#REG_SDIO_STOP_INTERNAL]    @\
 bic  r0,#1                              @ SDIO_STOP_INTERNAL clear bit0
 strh r0,[r9,#REG_SDIO_STOP_INTERNAL]    @/
 @- - -
 tst  r4,#1 << 27  @block mode          @\
 moveq r0,r2   @blklen                  @
 moveq r1,#1    @numblk                  @
 movne r0,#0x80  @blklen                  @
 movne r1,r2   @numblk                  @
 movne r12,r12,lsl #7 @len               @
 strh r0,[r9,#REG_SDIO_BLKLEN16]         @
 strh r1,[r9,#REG_SDIO_NUMBLK16]         @
.if try_sdio_data32_mode                @
 str  r0,[r9,#REG_SDIO_BLKLEN32]         @   @\this DISTURBS hardware
 str  r1,[r9,#REG_SDIO_NUMBLK32]         @   @/(even when in DATA16 mode?!)
.endif                                  @/

        .if try_sdio_ndma
                tst  r4,#1 << 27  @block mode
                beq  use_data16
               @- - -
                str  r0,[r9,#REG_SDIO_BLKLEN32]         @\this DISTURBS hardware
                str  r1,[r9,#REG_SDIO_NUMBLK32]         @/(even when in DATA16 mode?!)
                mov  r0,#0x002                           @\want DATA32 mode, step 1
                strh r0,[r9,#REG_SDIO_DATA_CTL]         @/
                ldr  r0,=0x402 @clear fifo, data32 mode @\want DATA32 mode, step 2
                str  r0,[r9,#REG_SDIO_IRQ32]            @/
                ldr  r3,=REGBASE_NDMA+0*0x1C
                tst  r4,#1 << 31 @param.writeflag
                add  r0,r9,#REG_SDIO_DATA32
                streq r0,[r3,#0x04]  @NDMAxSAD @\read
                streq r11,[r3,#0x08] @NDMAxDAD @/
                strne r11,[r3,#0x04] @NDMAxSAD @\write
                strne r0,[r3,#0x08]  @NDMAxDAD @/
                mov  r0,r12,lsr #2      @total words
                str  r0,[r3,#0x0c]  @NDMAxTNCT
                mov  r0,#0x80/4          @words per logical block
                str  r0,[r3,#0x10]  @NDMAxWCNT
                mov  r0,#0
                str  r0,[r3,#0x14]  @NDMAxBCNT
                ldreq r0,=(2 << 13)+(5 << 16)+(9 << 24)+(0 << 29)+(1 << 31)  @read
                ldrne r0,=(2 << 10)+(5 << 16)+(9 << 24)+(0 << 29)+(1 << 31)  @write
                str  r0,[r3,#0x1c]  @NDMAxCNT
                b    use_this
               @---
               use_data16:
                mov  r0,#0x000                           @\want DATA16 mode, step 1
                strh r0,[r9,#REG_SDIO_DATA_CTL]         @/
                ldr  r0,=0x400 @clear fifo, data32 mode @\want DATA16 mode, step 2
                str  r0,[r9,#REG_SDIO_IRQ32]            @/
               use_this:
        .endif

 @- - -
 tst  r4,#1 << 31 @param.writeflag      @\
 ldrne r0,=0x4c35 @cmd.wr   @\           @ send command
 ldreq r0,=0x5c35 @cmd.rd   @ SDIO_CMD   @
        tst  r4,#1 << 27 @block mode
        beq no_multi
        cmp r1,#1  @NUMBLK
        orrne r0,#0x2000 @multiblock   @XXX or is that used for NDMA with FIFO32?
       no_multi:
 strh r0,[r9,#REG_SDIO_CMD] @/           @/
@@busy_lop:                             @\
 ldr  r0,[r9,#REG_SDIO_IRQ_STAT]         @
 tst  r0,#0x7f0000 @ERR                   @ wait busy
 bne  error2                           @
 tst  r0,#1       @CMDRESPEND            @
 beq  busy_lop                        @/
 @- - -
 ldrh r0,[r9,#REG_SDIO_IRQ_STAT]         @\
 tst  r0,#1 << 22    @bit22, CMDTIMEOUT @ is that really needed here?
 bne  error_hw_timeout2                @/
@@@ ldr  r0,[r9,#REG_SDIO_REPLY] @\SDIO_REPLY (00001000h, ie. state = "dis"@ if it's "CSR")
@@@ and  r0,#0x00ff               @/

.if try_sdio_data32_mode        OR try_sdio_ndma
 ldrh r0,[r9,#REG_SDIO_DATA_CTL]         @\
 tst  r0,#2                              @
 beq  dta16                           @ redirect data16 / data32
 ldr  r0,[r9,#REG_SDIO_IRQ32]            @
 tst  r0,#2                              @
 beq  dta16                           @/
@- - -
dta32:  @---DATA32...
 ldr  r1,=REGBASE_NDMA+0*0x1C            @\
dta32_wait_ndma_done:                 @
 ldr  r0,[r9,#REG_SDIO_IRQ_STAT]         @
 tst  r0,#0x7f0000 @ERR                   @ wait for data NDMA done
 bne  error2                           @ (for both NDMA read or write)
 ldr  r0,[r1,#0x1c]  @NDMAxCNT            @
 tst  r0,#1 << 31                       @
 bne  dta32_wait_ndma_done            @/
 b    finish
.endif

@---
dta16:
 tst  r4,#1 << 31 @param.writeflag      @\check if read or write
 bne  dta16_write                     @/
dta16_wait_rxrdy:             @\              @\
 ldr  r0,[r9,#REG_SDIO_IRQ_STAT] @
 tst  r0,#0x7f0000 @ERR           @ wait for data @
 bne  error2                   @               @
 tst  r0,#0x1000000  @RXRDY16     @               @ read
 beq  dta16_wait_rxrdy        @/              @
 .if sdio_dma
   add  r12,#1 @round-up
   mov  r12,r12,lsr #1  @halfword count
   orr  r12,#0x81000000 @src=fix, 16bit, start
   mov  r3,#REGBASE_IO
   add  r0,r9,#REG_SDIO_DATA16
   str  r0,[r3,#REG_DMA3_SAD] @SAD
   str  r11,[r3,#REG_DMA3_DAD] @DAD
   str  r12,[r3,#REG_DMA3_CNT] @CNT
 .else
  dta16_rx_lop:                 @\              @
   ldrh r0,[r9,#REG_SDIO_DATA16]   @               @
   strh r0,[r11],#2                @ read data     @
   subs r12,#2                     @               @
   bhi  dta16_rx_lop            @/              @/
 .endif
 b    finish
@---
dta16_write:
dta16_wait_txrq:              @\              @\
 ldr  r0,[r9,#REG_SDIO_IRQ_STAT] @
 tst  r0,#0x7f0000 @ERR           @ wait          @
 bne  error2                   @               @
 tst  r0,#0x2000000  @TXRQ        @               @ write
 beq  dta16_wait_txrq         @/              @
 .if sdio_dma
   add  r12,#1 @round-up
   mov  r12,r12,lsr #1  @halfword count
   orr  r12,#0x00400000 @dst=fix, 16bit
   orr  r12,#0x80000000 @start
   mov  r3,#REGBASE_IO
   str  r11,[r3,#REG_DMA3_SAD] @SAD
   add  r0,r9,#REG_SDIO_DATA16
   str  r0,[r3,#REG_DMA3_DAD] @DAD
   str  r12,[r3,#REG_DMA3_CNT] @CNT
 .else
  dta16_tx_lop:                 @\              @
   ldrh r0,[r11],#2                @               @
   strh r0,[r9,#REG_SDIO_DATA16]   @ write data    @
   subs r12,#2                     @               @
   bhi  dta16_tx_lop            @/              @/
 .endif
 b    finish
@---
finish:
wait_dataend:                         @\
 ldr  r0,[r9,#REG_SDIO_IRQ_STAT]         @
 tst  r0,#0x7f0000 @ERR                   @ wait data end
 bne  error2                           @ (works BETTER with this!)
 tst  r0,#4     @DATAEND                 @
 beq  wait_dataend                    @/
@- - -
done3:
 pop  {r4-r12,pc}
@---
error2:
error_hw_timeout2:
 .if verbose_wifi_firmware
   bl   wrdot
 .endif
 mov  r0,#-1
 b    done3
