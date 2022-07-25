# $1=raw  $2=wav
sox -r 8000 -e signed -b 16 -c 1 $1 $2 
