FROM xanderhendriks/stm32cubeide:7.0

RUN apt-get -y update && \
    apt-get -y install python3 python3-pip && \
    ln -s $(which python3) /usr/bin/python

COPY requirements.txt .
RUN pip3 install -r requirements.txt

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
