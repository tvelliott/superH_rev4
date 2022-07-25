#cat good_p25_audio_fsk4.wav | pv -L 98500 -B 6400 | netcat -4 --mtu=1400 -u 192.168.1.150 8889
#cat ~/p25_large*.raw | pv -L 98500 -B 640 | netcat -4 --mtu=1400 -u 192.168.1.150 8889
cat $1 | pv -L 98500 -B 6400 | netcat -4 --mtu=1400 -u 192.168.1.150 8889
