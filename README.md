# mod.iot.sp.local

OPIL module: Local Sensing & Perception (Local SP)
Innovation Centre Nikola Tesla, written by Marija from May 6th 2018, contintributed by Goran, Jura, Petki and Ana

This is the local SP. The docker image is created by executing in the root folder of this package:

```
docker build -t "localsp:test" -f docker/localSPdocker/Dockerfile .
```

Test built docker container by starting `docker-compose up` from the test folder. In docker-compose.yml uncomment the line with #, and comment the line above:
```
  splocal:
    restart: always
    image: docker.ramp.eu/opil/opil.iot.sp.local:3.1.1
#    image: localsp:test
```

Documentation about the Local SP module can be found here <https://opil-documentation.readthedocs.io/en/latest/SP/Local_SP_Getting_Started.html>.
