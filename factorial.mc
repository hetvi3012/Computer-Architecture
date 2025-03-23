0x0  0x00000113  # addi x2, x0, 0   (i = 0)  
0x4  0x00000213  # addi x4, x0, 0   (swapped = 0)  
0x8  0x00400193  # addi x3, x0, 4   (array size - 1)  
0xC  0x00010463  # beq x2, x3, exit_outer  
0x10 0x00000513  # addi x10, x0, 0  (j = 0)  
0x14 0x00350563  # beq x10, x3, next_pass  
0x18 0x02052C03  # lw x24, 32(x10)  (load array[j])  
0x1C 0x02452C83  # lw x25, 36(x10)  (load array[j+1])  
0x20 0x019C8663  # blt x25, x24, swap  
0x24 0x00450513  # addi x10, x10, 4 (j++)  
0x28 0xFF1FF06F  # jal x0, -16      (back to loop)  
0x2C 0x02052C23  # sw x24, 36(x10)  (swap)  
0x30 0x02452C03  # sw x25, 32(x10)  
0x34 0x00100213  # addi x4, x0, 1   (swapped = 1)  
0x38 0xFE5FF06F  # jal x0, -28      (back to inner loop)  
0x3C 0x00110113  # addi x2, x2, 1   (i++)  
0x40 0x00410193  # addi x3, x2, 4  
0x44 0xFC9FF06F  # jal x0, -56      (back to outer loop)  
0x48 0x00000000  # TERM
