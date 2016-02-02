# roki-gazebo-patch

Roki (http://www.mi.ams.eng.osaka-u.ac.jp/software/roki.html) をシミュレータGazebo (www.gazebosim.org) で使用できるようにするためのパッチです。

インストール方法
================

Rokiのインストール
------------------
```
$ sudo add-apt-repository ppa:hrg/daily
$ sudo apt-get update
$ sudo apt-get install roki
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
$ cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_TESTS_COMPILATION:BOOL=False ../
$ make -j 4
$ make -j 4 install
```

physics.sdfの修正
-----------------
SDFファイルにrokiの設定を記述できるように、/usr/share/sdformat/1.5/physics.sdf を次のように修正します。
```
@@ -210,4 +210,29 @@
       </element>
     </element> <!-- End Constraints -->
   </element> <!-- ODE -->
+
+  <element name="roki" required="0">
+    <description>Roki specific physics properties</description>
+    <element name="solver_type" type="string" default="Volume" required="0">
+      <description>rkFDSetSolver() parameter. choose Vert or Volume</description>
+    </element>
+    <element name="contact_info" required="0">
+      <description></description>
+      <element name="compensation" type="double" default="1000" required="0">
+        <description>rkContactInfoRigidCreate() parameter</description>
+      </element>
+      <element name="relaxation" type="double" default="0.01" required="0">
+        <description>rkContactInfoRigidCreate() parameter</description>
+      </element>
+      <element name="static_friction" type="double" default="5.0" required="0">
+        <description>rkContactInfoRigidCreate() parameter</description>
+      </element>
+      <element name="friction" type="double" default="3.0" required="0">
+        <description>rkContactInfoRigidCreate() parameter</description>
+      </element>
+    </element>
+    <element name="debug_print" type="bool" default="false" required="0">
+      <description>enable debug print</description>
+    </element>
+  </element>
 </element> <!-- Physics -->
```

環境設定
========

~/.bashrcの設定
---------------
~/.bashrcに次の設定を追加して、Gazeboの実行に必要な環境変数を設定します。
```
  export LD_LIBRARY_PATH=$HOME/local/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
  export PATH=${HOME}/local/bin:${PATH}
  export PKG_CONFIG_PATH=$HOME/local/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH

  export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${HOME}/local/share/gazebo-6.0
  export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${HOME}/gazebo_models
```
~/.bashrcの設定後、sourceコマンドを使って~/.bashrcを読み込み設定を反映するか、またはbashを再起動して、設定した環境変数を反映してください。


動作確認
--------
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

シミュレーションの実行
======================

モデルファイルのダウンロード
----------------------------
次のコマンドを実行して、Gazeboで利用するモデルファイルをあらかじめダウンロードしておきます。
```
$ cd ~
$ hg clone https://bitbucket.org/osrf/gazebo_models　
```

