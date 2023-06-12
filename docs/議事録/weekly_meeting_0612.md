B4のNishimiya Tadashiといいます。これから私の研究テーマに関する発表を始めます

Im Nishimiya Tadashi from B4. I am going to start my presentation on my research topic.

研究手法は前回のミーティングと変わっていないので、今日は主にスケジュールやシステムについて話します

The research methodology has not changed since our last meeting, so I will omit some of the specific methods and today I will mainly talk about schedules and system architecture



## Research Theme & Motivation 

私の研究は、ロボットアームのペグインサーションなどのモーション生成を模倣学習を使って行うことです。

近年の最近の機械学習の目覚ましい変化を見ていて、動作計画に機械学習を使う研究に興味を持ち、機械学習を使ってよりロバストで正確な動きができるのではないかと考えたからです。また、模倣学習の技術を使ってさらに早く効率的に動作をおぼえられるのではないかと思いました。以上が私のこの研究のモチベーションです

My research involves the use of imitation learning to generate motion, such as peg insertion, for robotic arms.

I became interested in research using machine learning for motion planning after observing the recent remarkable develops in machine learning , and I thought that machine learning will be used to produce more robust and accurate movements. 



## Research Plan

そこで、私は今研究室で作成されている双椀アームを制作し、このロボットを本研究に使用しようと考えています。このロボットはダイレクトドライブの低ギア比のモーターを使っており、ロボット一台でティーチングとラーニングを同時に行い学習させたいと考えています。学習させるタスクはペグインサーションを考えています。最初は挿入動作のみを学習させ、徐々に探索などタスクを複雑化していきたいと思っています。

Therefore, I am planning to use two robot  arm that we are now creating for this study. This robot uses direct drive motors or low gear ratio motor boxes so that I would like to do teaching and learning tasks  with this single robot. The task to be learned is peg insertion using only sensors (which means I dont use cameras or other high functionality sensors). I would like the robot to learn only insertion movements at first, and then gradually increase the complexity of the task, such as exploration.  Eventually, I would like to try it with a soft object like a pocky or some food.



## System Architecture 2min

次に、スケジュールについて説明する前に全体のシステム構造を示します。

このロボットアームの学習システムは主に３つの大きなモジュールで構成されており、それぞれがプロセス間通信することでデータの受け渡しを行います。まず１つ目がロボットアームです。この中ではModbusサーバーが立ち上がっており、アームのそれぞれのセンサの値を読み込んだり、モーターの電流指令値を伝えるタスクを行っています。

次に、シミュレータです。シミュレータの制作は優先度が低く、卒業論文では使用しない可能性もあります。理由はこのロボットのCADモデルや各リンクの質量などのシミュレーションに必要なデータがなく、それらを測定してCADに置き換えるのは高コストのためです。ですが、ロボットアームの状態やログを表示するビジュアライザはあったほうが良いと考えているので、現在製作中です。

そして、最後にその２つのモジュールを制御するモジュールとして、学習モジュールがあります。ここで学習データを収集し、それを元にオンラインまたはオフラインで学習するシステムを組み込んでいます。そして、ビジュアライザ等と通信するこで状態を表示したりできるようにしようと考えています

Next, I'll show the overall system structure before discussing the schedule.

The learning system for this robotic arm consists mainly of three major modules, each of which passes data by means of interprocess communication. The first is the robot arm. In this, a Modbus server is up and running, which performs the tasks of reading the values of each of the arm's sensors and communicating the current command values for the motors.

Next is the simulator. The creation of the simulator is a low priority compared to the other two systems, and may not be used in the graduation thesis. The reason is that I do not have the CAD model of this robot, the mass of each link, and other data needed for simulation, and it would be expensive to measure them and replace them with CAD. However, we think it would be better to have a visualizer that displays the state and logs of the robot arm . Also, I want to detects when a robot tries to assume a physically impossible posture during learning steps, soI want to make a simple fowrward kinematics simulator and visualizer for this purpose.  

And finally, there is a learning module that controls those two modules. This is where I collect the learning data and incorporate a system that learns online or offline based on that data. 



## schedule and task  4min

最後にスケジュールについて話します。以下のようなスケジュールで７月の中間諮問まで進めようと考えています

まず、今はロボットアーム本体のトルク制御のプログラムを書いています。モーター２つが設置されただけの簡単な試験機を使い、電流制御を使ってバイラテラル動作をさせるのが目標です。EtherCATやモータードライバーの設定の問題で開発が止まることもあったのですが過去のプロジェクトで使っていたプログラムなどを参考にしながら製作中です。なお、過去のプロジェクトでバイラテラル動作をさせたときは追従性がとても悪く遅延があったそうですが、そのあたりの問題を解決したいと思っています

他のタスクとして、ロボットアームのモータードライバの取り替えなどがあります。現在のモーターでは手首周りのトルクが出ないので、先端部のモータを付け替える予定です。それと同時に、Peg Insertion用のハンドの先っぽも作ろうと考えています

以上のようなタスクを経て、中間諮問までにはロボットアーム全体の制御を目標にしています。例えば２つのロボットアームでバイラテラル制御を行うには、重力補償を考えたロボットアームの制御システム等が必要であり、それらのプログラムの開発にも今後取り組みたいと考えています。

そして、その後は学習データの収集または、オンラインでの模倣学習をおこない、少しずつ複雑なタスクにチャレンジしたいと考えています。



Finally, I will discuss the schedule. I am planning to proceed with the following schedule until the midterm exam 

First, I am now writing a program for torque control of the motor, using a simple test machine with only two motors installed. This goal is to move bilaterally using current controller. In the past projects, when executing  bilateral motion , the tracking performance was very poor and there were delays, and I would like to solve these problems.

Other tasks include replacing the motor drivers for the robot arm. The current motor does not produce enough torque around the wrist, so I plan to replace the motor at the tip. At the same time, I am also planning to make a hand tip for Peg Insertion!

After going through the above tasks, my goal is to control the entire robot arm by the mid-term exam. For example is the bilateral control

 in order to perform bilateral control with two robot arms, I need a control system for the robot arms that considers gravity compensation, etc. I would like to work on the development of those programs in the future.

After that, I am Then work on the learning task

The machine is designed to be more rigid compared to the power output of the motors, so it is designed to be moved online for long periods of time while learning.

After that, I would like to challenge more complex tasks little by little by collecting learning data or learning by imitation online.



### ending

以上で今後のスケジュールの説明を終わります。モータ制御まわりのトラブルやドライバーの開発元との連絡等に時間がかかってしまい、まだお見せできるような進捗がないので、中間諮問までにはそれっぽいアウトプットができるように進めていきます。

This concludes our schedule for the future. Due to the time required for troubles around motor control and communication with the company of the driver developer, I have not yet made good progress that I can show you, so I will proceed so that I can produce some output  by the mid-term exam.


Thank you for listening.

