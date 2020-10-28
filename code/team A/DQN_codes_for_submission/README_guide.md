Team B

오준호 교수님

20130440 이대호, 20150236 김현빈, 20150238 김현성, 20160499 이재호, 20160510 이정하, 20160636 조재경, 20160879 슈브라닐 센굽타

수정한 파일 명
dqn_model.py, DuelingDDQN_learn.py, DuelingDDQN_learn_pre.py, DuelingDDQN_main.py, DuelingDDQN_main_pre.py, Network_test.py, simulator.py, simulatorforNetwork_test.py

리뷰한 파일 명
simulator.py, dqn_learn.py, dqn_model.py, DuelingDDQN_learn.py, DuelingDDQN_learn_pre.py, DuelingDDQN_main.py, DuelingDDQN_main_pre.py, Network_test.py

기타 평가에 조교가 알아야할 내용
1. DQN training을 위해서 Dueling DQN, Double DQN을 병합한 DuelingDDQN을 활용했습니다.
이를 위해 수정한 파일은 dqn_model.py, DuelingDDQN_learn.py, DuelingDDQN_learn_pre.py, DuelingDDQN_main.py, DuelingDDQN_main_pre.py 입니다.

2. 처음 제공된 코드에서 아주 약간만 변경된 코드도 리뷰한 파일 명에 포함했습니다. 다만 기본적으로 주어진 코드를 따로 설명하지는 않고 바꾼 부분만을 중심으로 어떤 목적으로 해당 부분을 바꾸거나 추가했고 그를 달성하기 위한 기본적인 알고리즘이 무엇인지 설명을 했습니다. 특히 dqn_learn.py의 경우 과제 안내를 따라 일단 변경을 전혀 하지 않은 코드에 전체적으로 주석을 붙였습니다. 하지만 실제로 training을 할 때 쓰이는 것은 DuelingDDQN_learn.py, DuelingDDQN_learn_pre.py이고 해당 코드에는 dqn_learn.py로부터 변경된 부분만을 중심으로 주석을 달았습니다.

3. _pre가 붙은 파일들은 기존에 training한 모델을 불러와 추가로 training하기 위한 것입니다. 쉽게 말해 pre-trained model로 training하는 코드입니다.
따라서 기본 골격은 그것이 붙지 않은 파일과 같습니다. 그래서 pre-trained model로 training하기 위해 바뀐 부분이 무엇인지만 해당 코드 안에 주석으로 리뷰했습니다.

4. simulator.py와  simulatorforNetwork_test.py의 차이
실제 DQN training 시 사용되는 simulator.py와는 달리 simulatorforNetwork_test.py는 Network_test.py에서 모델 성능을 테스트할 때 불러와 사용하는 환경입니다.
따라서 실제 데모 상황을 최대한 반영하려 했습니다.(아래는 모두 simulatorforNetwork_test.py에서 변경된 사항, 해당 파일은 아래 변경된 사항을 제외하고 simulator.py와 기본적으로 같으므로 따로 리뷰하지 않았습니다.)
-로봇이 한 에피소드에서 취할 수 있는 최대 step수인 max_iter 숫자를 조금 더 크게 함(400 -> 500)
-로봇이 벽에 너무 가깝거나(6칸 범위 이내) 공에서 7칸 범위 이내일 때만 후진할 수 있는 simulator.py와는 달리 항상 후진이 가능하게 함
-에피소드가 끝나게 만드는 세 가지 조건 중 공이 하나도 안 보이는 채로 몇 step이상 지났을 때 에피소드를 종료하는 기준점을 높임. 로봇이 공을 하나도 못 찾은 채로 제자리에서 8바퀴 회전하고 86 스텝까지 이동해도 종료되지 않고 그 이상 움직여야 종료가 됨(해당 숫자는 '충분히 큰 임의의 수')

5. 학습 네트워크 평가는 'Model Evaluation.md'파일 안에 있습니다.

6. 조금 더 명확한 설명을 위해 blockdiagram 폴더 안에 blockdiagram를 작성했습니다.


# Usage

학습을 위한 명령어:

$ python3 DuelingDDQN_main.py

```

Pretrained model로 학습시키기 위한 명령어(Pretrained model 의 이름은 같은 폴더 내 'DuelingDDQN_net1029_test.pt'여야 합니다.):

$ python3 DuelingDDQN_main_pre.py

```

simualator를 테스트해보기 위한 명령어:

$ python3 simulator.py

```

학습된 네트워크를 테스트하기 위한 명령어(테스트하는 학습 네트워크의 이름은 같은 폴더  'Trained_Model.pt'여야 합니다.):
$ python3 Network_test.py

```


The model, basic DQN and newly defined DuelingDQN is defined in `dqn_model.py`

The algorithm is defined in `DuelingDDQN_learn.py` and `DuelingDDQN_learn_pre.py`

The running script and hyper-parameters are defined in `DuelingDDQN_main.py` and `DuelingDDQN_main_pre.py`

related paper link https://www.nature.com/articles/nature14236.pdf

-------------------------------------------------------------------
if you want to use tensorboard in pythorch(monitoring DNN)

1. install  tensorboardX
```
pip install tensorboardX
```
2. isntall tensorflow
```
pip install tensorflow
```
3. launch tensorboard at your directory
```
cd dqn_learning_code
tensorboard --logdir runs
```
4. copy the address and paste the address at webbrowser

ex) http://humanlab-Z370-AORUS-Gaming-3:6006

# Sponsors
----------------------------------------------------------------------------------------------------
![sponsor_image](https://user-images.githubusercontent.com/47877833/54169800-90c45280-44b7-11e9-9878-3048ad1050c0.png)
