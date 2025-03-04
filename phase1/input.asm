.text
start:      add x1, x2, x3
            sub x4, x5, x6
            and x7, x8, x9
            sll x10, x11, x12
            or x13, x14, x15
            slt x16, x17, x18
            sra x19, x20, x21
            srl x22, x23, x24
            xor x25, x26, x27
            mul x28, x29, x30
            div x1, x2, x3
            rem x4, x5, x6
            addi x7, x8, 10
            andi x9, x10, 20
            ori x11, x12, 30
            lb x13, 0(x14)
            lh x15, 4(x16)
            lw x17, 8(x18)
            jalr x19, 0(x20)
            beq x21, x22, loop
            bne x23, x24, exit
            blt x25, x26, func
            bge x27, x28, done
            auipc x29, 1000
            lui x30, 2000
            jal x31, start

loop:       add x1, x1, x1
exit:       sub x2, x2, x2
func:       and x3, x3, x3
done:       or x4, x4, x4

.data
data1:      .byte 10, 20, 30, 40
            .half 100, 200
            .word 300, 400
            .dword 5000, 6000
            .asciz "Hello, World!"
