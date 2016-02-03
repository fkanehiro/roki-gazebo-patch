# roki-gazebo-patch

Robot Kinetics Library Roki (http://www.mi.ams.eng.osaka-u.ac.jp/software/roki.html) をシミュレータGazebo (www.gazebosim.org) で使用できるようにするためのパッチです。

インストール方法
================

Rokiのインストール
------------------
```
$ sudo add-apt-repository ppa:hrg/daily
$ sudo apt-get update
$ sudo apt-get install roki-dev
```

Gazeboのビルド環境整備
----------------------
```
$ sudo sh -c "echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main\" > /etc/apt/sources.list.d/gazebo-latest.list"
$ wget --quiet http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install build-essential libtinyxml-dev libtbb-dev libxml2-dev libqt4-dev pkg-config  libprotoc-dev libfreeimage-dev libprotobuf-dev protobuf-compiler libboost-all-dev freeglut3-dev cmake libogre-1.8-dev libtar-dev libcurl4-openssl-dev libcegui-mk2-dev libopenal-dev mercurial xsltproc ruby-dev libignition-math2 libignition-math2-dev libignition-transport0-dev uuid-dev graphviz graphviz-dev   libtinyxml2-dev libusb-1.0  libswscale-dev libavformat-dev libavcodec-dev libavutil-dev
```

SDFormat3のインストール
-----------------------
```
$ mkdir ~/gazebo_source
$ cd ~/gazebo_source/
$ hg clone https://bitbucket.org/osrf/sdformat
$ cd sdformat/
$ hg up sdformat3_3.7.0
$ mkdir build
$ cd build/
$ cmake ../
$ make -j 4
$ sudo make install
```

Gazeboのソースコードの取得
--------------------------
```
$ cd ~/gazebo_source/
$ hg clone https://bitbucket.org/osrf/gazebo
$ cd gazebo/
$ hg up gazebo6_6.0.0
```

パッチの適用
------------
```
$ cd ~/gazebo_source/gazebo
$ patch -u -p1 <path to this project>/src/patch-gazebo-roki-20160111.diff
```

ビルド
------
```
$ cd ~/gazebo_source/gazebo/
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_TESTS_COMPILATION:BOOL=False -DCMAKE_INSTALL_PREFIX=/usr/local ../
$ make -j 4
$ make -j 4 install
```
gzserverのヘルプを表示して、対応しているシミュレータ動力学計算エンジンの一覧に Rokiが追加されていることを確認してください。
```
$ gzserver -h
gzserver -- Run the Gazebo server.

`gzserver` [options] <world_file>

Gazebo server runs simulation and handles commandline options, 
starts a Master, runs World update and sensor generation loops.

Options:
  -v [ --version ]              Output version information.
  --verbose                     Increase the messages written to the terminal.
  -h [ --help ]                 Produce this help message.
  -u [ --pause ]                Start the server in a paused state.
  -e [ --physics ] arg          Specify a physics engine 
                                (ode|bullet|dart|simbody|roki).
  -p [ --play ] arg             Play a log file.
  -r [ --record ]               Record state data.
  --record_encoding arg (=zlib) Compression encoding format for log data 
                                (zlib|bz2|txt).
  --record_path arg             Absolute path in which to store state data
  --seed arg                    Start with a given random number seed.
  --iters arg                   Number of iterations to simulate.
  --minimal_comms               Reduce the TCP/IP traffic output by gzserver
  -s [ --server-plugin ] arg    Load a plugin.
  -o [ --profile ] arg          Physics preset profile name from the options in
                                the world file.
```


physics.sdfの修正
-----------------
SDFファイルにrokiの設定を記述できるように、/usr/share/sdformat/1.5/physics.sdf にパッチをあてます。
```
$ cd /usr/share/sdformat/1.5
$ sudo patch < <path to this project>/src/physics.sdf.patch
```

シミュレーションの実行
======================
次のように-eオプションでrokiを指定してGazeboを起動しシミュレーションを開始します。
```
gazebo --pause -e roki <path to this project>/worlds/test10_box10.world
```
