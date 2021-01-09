# mod.iot.sp.local

OPIL module: Local Sensing & Perception (Local SP)
Innovation Centre Nikola Tesla, written by Marija from May 6th 2018, contintributed by Goran, Jura, Petki and Ana

This is the local SP. The docker image is created by executing in the root folder of this package:

```
docker build -t "localsp:test" -f docker/localSPdocker/Dockerfile .
```

Test built docker container by starting `docker-compose up` from the test folder. Make sure you started the OCB before. You can start it from the Central SP by starting `docker-compose up` from the folder `test/docker_compose_files/Central_SP_docker.

Documentation about the Local SP module can be found here <https://opil-documentation.readthedocs.io/en/latest/SP/Local_SP_Getting_Started.html>.
