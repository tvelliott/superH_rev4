netcat -l -u -p 8889 | baudline -stdin -stdout -samplerate 48e3 -channels 1 -format le16 >$1 
