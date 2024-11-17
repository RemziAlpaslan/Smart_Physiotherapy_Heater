# Smart_Physiotherapy_Heater

## Bir bash dosyası oluşturdum

nano proje.sh

##  İçine proje.py yi çalıştıran kodu yazdım

!/bin/bash

sleep 45

lxterminal -e python3 /home/pi/Desktop/proje.py

##  Kaydedip çıktım ve bash dosyasına izin verdim.

chmod +x proje.sh

##  proje.sh dosyasının yolunu buldum

realpath proje.sh

##  Bu yolu otomatik başlatmanın olduğu yerde çalıştırdım.

sudo nano /etc/xdg/lxsession/LXDE-pi/autostart

##  açılan pencerenin en altına bu kodu koyun

@lxterminal -e bash /home/pi/proje.sh
