KP=50
KI=0.01
KD=5
lspeed=30
rspeed=30
speed_left=16.1
speed_right=18.3
lsp_error=13.89
rsp_error=11.7
dif_lsp_error=13.9
dif_rsp_error=11.7
sum_lsp_error=-79099
sum_rsp_error=-104099

print "PID:",KP*lsp_error+KI*sum_lsp_error+dif_lsp_error+abs(lspeed)
