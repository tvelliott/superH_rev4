# send_p25_voice_8khz 1
netcat -l -u -p 8889 | baudline -stdin -stdout -samplerate 48e3 -channels 1 -format le16 | lame -r -s 8 -V 9 --vbr-new -mm -h -q 0 - $1
