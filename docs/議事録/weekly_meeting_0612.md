B4のNishimiya Tadashiといいます。これから私の研究テーマに関する発表を始めます

Im Nishimiya Tadashi from B4. I am going to start my presentation on my research topic.
I know it's not very detailed, but thank you in advance for listening


## Research Theme & Motivation 

私はロボットのモーション生成に興味があります。そして、最近の機械学習の目覚ましい変化を見ていて、動作計画に機械学習を使う研究に興味を持つようになりました。従来の人の手で計画を定義するのではなく、機械学習である程度動的に経路生成することでよりロバストで人間らしい動きができるのではないかと考えたからです。
また、ティーチングや模倣学習の技術を使って効率的に学習させることで、さらに早く効率的に動作をおぼえられるのではないかと思いました。以上が私のこの研究のモチベーションです


I am interested in motion planning for robots. And, seeing the recent remarkable changes in machine learning, I have become interested in research on using machine learning for motion planning or for robotics. 

For example, supervised learning methods based on behavior cloning (BC) suffer from distributional shifts, where agents greedily imitate the teached behavior and move different motions from the demonstration state due to error accumulation.  But, generative adversarial imitation learning (GAIL) overcome this problem by training learning agents to match demonstrations over a long horizon.

For, such implovements of machine learning, 

This is because I thought that instead of defining plans by programming as in the past or mathematically, machine learning could be used to generate pathways dynamically to produce more robust and smooth motion.

I also thought that by using teaching and imitation learning techniques to learn efficiently, it would be possible to learn movements even faster and more efficiently. I thought this might be a solution to the problem of machine learning taking a long time to learn, and I also thought it might be widely useful in industry, etc.　This is my motivation for this research.


## Related Works

過去の研究でティーチングを用いた模倣学習は多くの研究で行われています。中でも良い結果を残したものとして以下のような研究を調べてみました。
まず、この研究室で行われた研究で、双椀アームを使ってバナナの皮を剥くタスクを生成することに成功しました。この手法がティーチングと行動クローニングを行い、様々なバナナの形状についてロバストなアクションを生成することができました。別の研究ではペグの画像を認識して複数のペグに対して挿入するアクションを作ることに成功しました。この研究の入力は画像ですが、ロボットアームのトルクを入力に使用した研究例として、いくつかのペグ員サーションの例があります。これらは良い結果を残していますが、ハンドの先端にトルクセンサをつけたもので、関節にかかるモーメントなどは考慮されていませんでした。

In the past, imitation learning has been used in many studies. I examined the following studies.  First, a study conducted in this lab successfully generated a task to peel a banana using a twin arm robots. The method performed teaching and action cloning and was able to generate robust actions for a variety of banana shapes. Another study successfully recognized images of pegs and created actions to insert for multiple types of pegs. While the input for this study was an image, there are several examples of studies that used the torque of a robotic arm as input for several peg-insertions. These had good results, and they were based on torque sensors attached to the tip of the hand, 
and did not take into account the moments applied to the joints.
However, few studies have used joint torques and imitation learning for peg insertion tasks.

## Research Plan

そこで、私は今研究室で作成されている双椀アームを制作し、このロボットを本研究に使用しようと考えています。このロボットはダイレクトドライブ、（関節にブラシレスモーターが直接接続されている形状）をしている一般的な産業用ロボットとは異なる形状をしています。またギアを使っているところもギア比が低く、柔軟な動作が可能になっています。以下の動画がその例です。このようにモーターの電源を切っている段階では自由に動かすことができるので、このロボット一台でティーチングとラーニングを同時に行い、学習させたいと考えています。学習させるタスクはペグインサーションを考えています。最初は挿入動作のみを学習させ、徐々に探索などタスクを複雑化していきたいと思っています


Therefore, I am planning to use a robot arm that is currently under construction  in our lab and use this robot for this research. Currently, there may be a problem with output torque, etc., and we would like to improve this area.  This robot has a different type of design from general often used  industrial robots 
that have direct drive (brushless motors directly connected to the joints). Also, the gears used in this robot have a low gear ratio , Very low gear ratio, about 1:4 to 1:5..  So this robot can move smoothly ,   and flexible. The following video is an example. Since the robot can move freely when the motors are turned off, we would like to use this single robot for teaching and learning at the same time. 

In Previous researches, teaching tasks using with in VR or using human arms had many problems. For example, the actual range of motion of the robot and the range of motion of the human VR space are different, so unnecessary areas are also learned or . Another problem is this method may have to solve  inverse kinematics to convert human arm space to robotics, which takes even more time to learn.so, I believe that using the same robot and same model to generate training data will allow us to focus more on the learning task, which will result in faster learning.

The task to be learned is peg insertion. At first, we would like the robot to learn only insertion movements, and then gradually increase the complexity of the task, such as searching.We want to achieve robust motion generation for various peg states and locations using joint angle torques and other inputs.



## System Architecture

次に、スケジュールについて説明する前に全体のシステム構造を示します。

このロボットアームの学習システムは主に３つの大きなモジュールで構成されており、それぞれがプロセス間通信することでデータの受け渡しを行います。まず１つ目がロボットアームです。この中ではModbusサーバーが立ち上がっており、アームのそれぞれのセンサの値を読み込んだり、モーターの電流指令値を伝えるタスクを行っています。

次に、シミュレータです。シミュレータの制作は優先度が低く、卒業論文では使用しない可能性もあります。理由はこのロボットのCADモデルや各リンクの質量などのシミュレーションに必要なデータがなく、それらを測定してCADに置き換えるのは高コストのためです。ですが、ロボットアームの状態やログを表示するビジュアライザはあったほうが良いと考えているので、現在製作中です。

そして、最後にその２つのモジュールを制御するモジュールとして、学習モジュールがあります。ここで学習データを収集し、それを元にオンラインまたはオフラインで学習するシステムを組み込んでいます。そして、ビジュアライザ等と通信するこで状態を表示したりできるようにしようと考えています



## schedule and task 

最後にスケジュールについて話します。以下のようなスケジュールで７月の中間諮問まで進めようと考えています















より直接的　司令してトルクがそのままでるという
壊れやすいオブジェクト
リジッドなもの→柔らかい
リジッドすぎると壊しちゃったみたいなものも怒りそうなので、力で押し付けてっていうのがそのままできる。
位置司令をしてさきっぽの弾性で制御しているのが、それをする必要がなくて望む力でそのまま押し当てられる

手先に力センサついてるので　


まる棒→資格、


以上が参考文献です
聞いてくれてありがとうございました

These are the references
Thank you for listening.

