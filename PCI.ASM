_TEXT segment dword public 'CODE'
    assume cs:_TEXT

    
    
    .386p

    




; general notes:
;
; arguments are pushed from right to left
; when the C calling convention is used
;






; int far cdecl _pci_is_bios_present( void )
;
; returns 1 in AX if PCI BIOS is present
; returns 0 in AX otherwise
;
_pci_is_bios_present proc far

    ; save the caller's registers
    ;
    push edx

    ; AH = PCI function ID
    ; AL = PCI BIOS presence query
    ;
    mov ax, 0B101h
    int 1Ah

    ; should now have "PCI " in EDX register
    ;
    cmp edx, " ICP"             ; byte swapped for little-endian
    setz al                     ; AL = 1 IFF zero flag set
    cbw                         ; AX = AL w/ sign extension

    ; restore the caller's registers
    ;
    pop edx

    ret

_pci_is_bios_present endp






; unsigned short far _cdecl pci_get_device_handle(
;     unsigned short device_id,     // BP + 6
;     unsigned short vendor_id,     // BP + 8
;     unsigned short index          // BP + 10
; )
;
; returns the handle of the Nth device with the
; desired device ID and vendor ID
;
; call this function multiple times, incrementing
; the index from zero onward, until it returns
; 0xFFFF indicating no (more) cards matched
;
; note: for Trancell Systems PCI-155 ATM network
; interface card, the device ID is 11D3h and the
; vendor ID is 0002h
;
_pci_get_device_handle proc far

    ; set up the stack frame
    ;
    push bp
    mov bp, sp

    ; save the caller's registers
    ;
    push cx
    push si

    ; AH = PCI function ID
    ; AL = find device
    ;
    mov ax, 0B102h
    mov cx, [bp + 6]
    mov dx, [bp + 8]
    mov si, [bp + 10]
    int 1Ah

    jnc handle_ok
    mov ax, 0FFFFh
    jmp handle_done

handle_ok:
    ; move the handle into AX for return to C
    ;
    ; the minimal doc I have says that device handle
    ; is just the upper 5 bits (7:3) of BL, but
    ; disassembly of TESTCM.EXE provided by Trancell
    ; uses entire contents of BX to pass to its next
    ; call to PCI BIOS, so I assume that is where
    ; the bits should stay
    ;
    movzx ax, bl

handle_done:
    ; restore the caller's registers
    ;
    pop si
    pop cx

    ; clean up the stack frame
    ;
    pop bp

    ; caller cleans up the stack themselves
    ;
    ret

_pci_get_device_handle endp






; Bit32 far _cdecl pci_read_config_32(
;     Bit16 device_handle,     // BP + 6
;     Bit16 offset             // BP + 8
; )
;
; returns the 32-bit configuration register
; on the PCI device with the given handle
; at the given offset
;
; errors are ignored
;
_pci_read_config_32 proc far

    ; set up the stack frame
    ;
    push bp
    mov bp, sp

    ; save the caller's registers
    ;
    push bx
    push ecx
    push di

    ; AH = PCI function ID
    ; AL = read configuration DWORD
    ; BX 15:8 = bus number
    ;     7:3 = device ID
    ;     2:0 = function number (subdevice)
    ; DI = byte offset into config space
    ;
    mov ax, 0B10Ah
    mov bx, [bp + 6]
    mov di, [bp + 8]
    int 1Ah

    ; move the value into DX:AX for return to C language
    ;
    mov ax, cx
    shr ecx, 16
    mov dx, cx

    ; restore the caller's registers
    ;
    pop di
    pop ecx
    pop bx

    ; clean up the stack frame
    ;
    pop bp

    ; caller cleans up the stack themselves
    ;
    ret

_pci_read_config_32 endp








; Bit16 far _cdecl pci_read_config_16(
;     Bit16 device_handle,     // BP + 6
;     Bit16 offset             // BP + 8
; )
;
; returns the 16-bit configuration register
; on the PCI device with the given handle
; at the given offset
;
; errors are ignored
;
_pci_read_config_16 proc far

    ; set up the stack frame
    ;
    push bp
    mov bp, sp

    ; save the caller's registers
    ;
    push bx
    push cx
    push di

    ; AH = PCI function ID
    ; AL = read configuration WORD
    ; BX 15:8 = bus number
    ;     7:3 = device ID
    ;     2:0 = function number (subdevice)
    ; DI = byte offset into config space
    ;
    mov ax, 0B109h
    mov bx, [bp + 6]
    mov di, [bp + 8]
    int 1Ah

    ; move the value into AX for return to C language
    ;
    mov ax, cx

    ; restore the caller's registers
    ;
    pop di
    pop cx
    pop bx

    ; clean up the stack frame
    ;
    pop bp

    ; caller cleans up the stack themselves
    ;
    ret

_pci_read_config_16 endp








; Bit8 far _cdecl pci_read_config_8(
;     Bit16 device_handle,     // BP + 6
;     Bit16 offset             // BP + 8
; )
;
; returns the 8-bit configuration register
; on the PCI device with the given handle
; at the given offset
;
; errors are ignored
;
_pci_read_config_8 proc far

    ; set up the stack frame
    ;
    push bp
    mov bp, sp

    ; save the caller's registers
    ;
    push bx
    push cx
    push di

    ; AH = PCI function ID
    ; AL = read configuration BYTE
    ; BX 15:8 = bus number
    ;     7:3 = device ID
    ;     2:0 = function number (subdevice)
    ; DI = byte offset into config space
    ;
    mov ax, 0B108h
    mov bx, [bp + 6]
    mov di, [bp + 8]
    int 1Ah

    ; move the value into AL for return to C language
    ;
    mov al, cl

    ; restore the caller's registers
    ;
    pop di
    pop cx
    pop bx

    ; clean up the stack frame
    ;
    pop bp

    ; caller cleans up the stack themselves
    ;
    ret

_pci_read_config_8 endp








; void far _cdecl pci_write_config_32(
;     Bit16 device_handle,     // BP + 6
;     Bit16 offset,            // BP + 8
;     Bit32 value              // BP + 10
; )
;
; sets the 32-bit configuration register
; on the PCI device with the given handle
; at the given offset to the given value
;
; errors are ignored
;
_pci_write_config_32 proc far

    ; set up the stack frame
    ;
    push bp
    mov bp, sp

    ; save the caller's registers
    ;
    push ax
    push bx
    push ecx
    push di

    ; AH = PCI function ID
    ; AL = write configuration DWORD
    ; BX 15:8 = bus number
    ;     7:3 = device ID
    ;     2:0 = function number (subdevice)
    ; DI = byte offset into config space
    ; ECX = value to write
    ;
    mov ax, 0B10Dh
    mov bx, [bp + 6]
    mov di, [bp + 8]
    mov ecx, [bp + 10]
    int 1Ah

    ; restore the caller's registers
    ;
    pop di
    pop ecx
    pop bx
    pop ax

    ; clean up the stack frame
    ;
    pop bp

    ; caller cleans up the stack themselves
    ;
    ret

_pci_write_config_32 endp








; void far _cdecl pci_write_config_16(
;     Bit16 device_handle,     // BP + 6
;     Bit16 offset,            // BP + 8
;     Bit16 value              // BP + 10
; )
;
; sets the 16-bit configuration register
; on the PCI device with the given handle
; at the given offset to the given value
;
; errors are ignored
;
_pci_write_config_16 proc far

    ; set up the stack frame
    ;
    push bp
    mov bp, sp

    ; save the caller's registers
    ;
    push ax
    push bx
    push cx
    push di

    ; AH = PCI function ID
    ; AL = write configuration WORD
    ; BX 15:8 = bus number
    ;     7:3 = device ID
    ;     2:0 = function number (subdevice)
    ; DI = byte offset into config space
    ; CX = value to write
    ;
    mov ax, 0B10Ch
    mov bx, [bp + 6]
    mov di, [bp + 8]
    mov cx, [bp + 10]
    int 1Ah

    ; restore the caller's registers
    ;
    pop di
    pop cx
    pop bx
    pop ax

    ; clean up the stack frame
    ;
    pop bp

    ; caller cleans up the stack themselves
    ;
    ret

_pci_write_config_16 endp








; void far _cdecl pci_write_config_8(
;     Bit16 device_handle,     // BP + 6
;     Bit16 offset,            // BP + 8
;     Bit8 value               // BP + 10
; )
;
; sets the 8-bit configuration register
; on the PCI device with the given handle
; at the given offset to the given value
;
; errors are ignored
;
_pci_write_config_8 proc far

    ; set up the stack frame
    ;
    push bp
    mov bp, sp

    ; save the caller's registers
    ;
    push ax
    push bx
    push cx
    push di

    ; AH = PCI function ID
    ; AL = write configuration BYTE
    ; BX 15:8 = bus number
    ;     7:3 = device ID
    ;     2:0 = function number (subdevice)
    ; DI = byte offset into config space
    ; CL = value to write
    ;
    mov ax, 0B10Bh
    mov bx, [bp + 6]
    mov di, [bp + 8]
    mov cl, [bp + 10]
    int 1Ah

    ; restore the caller's registers
    ;
    pop di
    pop cx
    pop bx
    pop ax

    ; clean up the stack frame
    ;
    pop bp

    ; caller cleans up the stack themselves
    ;
    ret

_pci_write_config_8 endp








public _pci_is_bios_present
public _pci_get_device_handle
public _pci_read_config_32
public _pci_read_config_16
public _pci_read_config_8
public _pci_write_config_32
public _pci_write_config_16
public _pci_write_config_8






_TEXT ends

    end
