#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <algorithm>
#include <unistd.h>
#include <math.h>

#define PI 3.1415926535

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )


//ControllerのサブクラスMoveControllerの宣言します
class RobotController : public Controller {
public:

  void onInit(InitEvent &evt);
  double onAction(ActionEvent&);
  void onRecvMsg(RecvMsgEvent &evt);
  void onCollision(CollisionEvent &evt);
  bool recognizeTrash(Vector3d &pos, std::string &name);
  std::string getPointedTrashName(std::string entName);

  // エージェントが指差している方向にあるオブジェクトの名前を取得します
  std::string getPointedObjectName(std::string entName);

  /* @brief  位置を指定しその方向に回転を開始し、回転終了時間を返します
   * @param  pos 回転したい方向の位置
   * @param  vel 回転速度
   * @param  now 現在時間
   * @return 回転終了時間
   */
  double rotateTowardObj(Vector3d pos, double vel, double now);

  /* @brief  位置を指定しその方向に進みます
   * @param  pos   行きたい場所
   * @param  vel   移動速度
   * @param  range 半径range以内まで移動
   * @param  now   現在時間
   * @return 到着時間
   */
  double goToObj(Vector3d pos, double vel, double range, double now);

private:
  // Takeitの対称から除外するオブジェクトの名前
  std::vector<std::string> exobj;
  std::vector<std::string> extrash;
  RobotObj *m_my;
  Vector3d m_tpos;

  //状態
  /***
   * @brief ロボットの状態
   * 0 "TAKEIT"の指示を待っている状態
   * 1 取りに行くオブジェクトを特定中
   * 2 オブジェクトに向けて移動中
   * 3 オブジェクトを取得し戻る途中
   */
  int m_robotState;
  int m_state;

  // エージェントの正面の方向ベクトル通常はz軸の正の方向と考える
  Vector3d m_defaultNormalVector;

  // 取りにいくオブジェクト名
  std::string m_pointedObject;


// pointed trash
  std::string m_pointedtrash;
  // 指示を受けたアバター名
  std::string m_avatar;

  // onActionの戻り値
  double m_onActionReturn;

  // 1stepあたりの移動距離
  double m_speedDelta;

  // ゴミの名前
  std::string m_tname;
  // ゴミ候補オブジェクト
  std::vector<std::string> m_trashes;
  // ゴミ箱オブジェクト
  std::vector<std::string> m_trashboxs;

  // 車輪の角速度
  double m_vel;

  // 関節の回転速度
  double m_jvel;

  // 車輪半径
  double m_radius;

  // 車輪間距離
  double m_distance;

  // 移動終了時間
  double m_time;
  double m_time_LA1;
  double m_time_LA4;
  double m_time_RA1;
  double m_time_RA4;

  // 初期位置
  Vector3d m_inipos;

  Vector3d pos_a;
  Vector3d pos_b;

  // grasp中かどうか
  bool m_grasp;

  // ゴミ認識サービス
  BaseService *m_recogSrv;

  // サービスへのリクエスト複数回送信を防ぐ
  bool m_sended;

  std::string msg_ob;
  std::string msg_trash;

};

void RobotController::onInit(InitEvent &evt)
{
  m_robotState = 0;
  //エージェントの正面の定義はz軸の正の向きを向いていると仮定する
  m_defaultNormalVector = Vector3d(0,0,1);
  m_onActionReturn = 1.0;
  m_speedDelta = 2.0;

  m_avatar = "man_000";

  m_my = getRobotObj(myname());

  // 初期位置取得
  //m_my->getPosition(m_inipos);
  m_my->getPartsPosition(m_inipos,"RARM_LINK2");

  pos_a = Vector3d(100, 30, -10);//
  pos_b = Vector3d(0, 30, -10);  ///

  // 車輪間距離
  m_distance = 10.0;

  // 車輪半径
  m_radius  = 10.0;

  // 移動終了時間初期化
  m_time = 0.0;
  m_time_LA1 = 0.0;
  m_time_LA4 = 0.0;
  m_time_RA1 = 0.0;
  m_time_RA4 = 0.0;

  // 車輪の半径と車輪間距離設定
  m_my->setWheel(m_radius, m_distance);
  m_state = 20;

  srand((unsigned)time( NULL ));

  // 車輪の回転速度
  m_vel = 0.3;

  // 関節の回転速度
  m_jvel = 0.6;

  // grasp初期化
  m_grasp = false;
  m_recogSrv = NULL;
  m_sended = false;

  // ここではゴミの名前が分かっているとします
  //m_trashes.push_back("petbottle_0");
  //m_trashes.push_back("petbottle_1");
  //m_trashes.push_back("petbottle_2");
  //m_trashes.push_back("petbottle_3");
  m_trashes.push_back("petbottle");
  //m_trashes.push_back("banana");
  //m_trashes.push_back("chigarette");
  //m_trashes.push_back("chocolate");
  //m_trashes.push_back("mayonaise_0");
  //m_trashes.push_back("mayonaise_1");
  m_trashes.push_back("mugcup");
  //m_trashes.push_back("can_0");
  //m_trashes.push_back("can_1");
  m_trashes.push_back("can");
  //m_trashes.push_back("can_3");

  // ゴミ箱登録
  m_trashboxs.push_back("recycle");
  m_trashboxs.push_back("burnable");
  m_trashboxs.push_back("unburnable");
  m_trashboxs.push_back("wagon");

}

double RobotController::onAction(ActionEvent &evt)
{
	switch(m_state){

  // 初期姿勢を設定 seting initial pose
  case 0: {
    //broadcastMsgToSrv("Let's start the clean up task\n");
    sendMsg("VoiceReco_Service","Let's start the clean up task\n");
    double angL1 =m_my->getJointAngle("LARM_JOINT1")*180.0/(PI);
    double angL4 =m_my->getJointAngle("LARM_JOINT4")*180.0/(PI);
    double angR1 =m_my->getJointAngle("RARM_JOINT1")*180.0/(PI);
    double angR4 =m_my->getJointAngle("RARM_JOINT4")*180.0/(PI);
    double thetaL1 = -20-angL1;
    double thetaL4 = -160-angL4;
    double thetaR1 = -20-angR1;
    double thetaR4 = -160-angR4;
    if(thetaL1<0) m_my->setJointVelocity("LARM_JOINT1", -m_jvel, 0.0);
    else m_my->setJointVelocity("LARM_JOINT1", m_jvel, 0.0);
    if(thetaL4<0) m_my->setJointVelocity("LARM_JOINT4", -m_jvel, 0.0);
    else m_my->setJointVelocity("LARM_JOINT4", m_jvel, 0.0);
    if(thetaR1<0) m_my->setJointVelocity("RARM_JOINT1", -m_jvel, 0.0);
    else m_my->setJointVelocity("RARM_JOINT1", m_jvel, 0.0);
    if(thetaR4<0) m_my->setJointVelocity("RARM_JOINT4", -m_jvel, 0.0);
    else m_my->setJointVelocity("RARM_JOINT4", m_jvel, 0.0);
    m_time_LA1 = DEG2RAD(abs(thetaL1))/ m_jvel + evt.time();
    m_time_LA4 = DEG2RAD(abs(thetaL4))/ m_jvel + evt.time();
    m_time_RA1 = DEG2RAD(abs(thetaR1))/ m_jvel + evt.time();
    m_time_RA4 = DEG2RAD(abs(thetaR4))/ m_jvel + evt.time();
    m_state = 1;
    break;
  }
  // 初期姿勢に移動 moving initial pose
  case 1: {
    if(evt.time() >= m_time_LA1) m_my->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
    if(evt.time() >= m_time_LA4) m_my->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
    if(evt.time() >= m_time_RA1) m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
    if(evt.time() >= m_time_RA4) m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
    if(evt.time() >= m_time_LA1 && evt.time() >= m_time_LA4
    && evt.time() >= m_time_RA1 && evt.time() >= m_time_RA4){
	// 位置Aの方向に回転を開始します setting position a for rotating
	//broadcastMsgToSrv("Moving to the table");
	m_time = rotateTowardObj(pos_a, m_vel, evt.time());
	m_state = 2;
    }
    break;
  }
  // 位置Aの方向に回転 rotating to position a
  case 2: {
    // 回転終了
    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);
      // 位置Aに移動します setting position a for moving
      m_time = goToObj(pos_a, m_vel*4, 0.0, evt.time());
      m_state = 3;
    }
    break;
  }
  // 位置Aに移動 moving to position a
  case 3: {
    // 位置Aに到着
    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);
      // 位置Bの方向に回転を開始します setting position b for rotating
      m_time = rotateTowardObj(pos_b, m_vel, evt.time());
      m_state = 4;
    }
    break;
  }
  // 位置Bの方向に回転 rotating to position b
  case 4: {
    // 回転終了
    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);
      // 位置Bに移動します setting position b for moving
      m_time = goToObj(pos_b, m_vel*4, 0.0, evt.time());
      m_state = 5;
    }
    break;
  }
  // 位置Bに移動 moving to position b
  case 5: {
    // 位置Bに到着
    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);
      // テーブルの方向に回転を開始します setting table position for rotating
      SimObj *table = getObj("table_0");
      Vector3d pos;
      table->getPosition(pos);
      m_time = rotateTowardObj(pos, m_vel, evt.time());
      m_state = 6;
    }
    break;
  }
  // テーブルの方向に回転 rotating to table
  case 6: {
    // 回転終了
    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);
      // ゴミがある場所と名前を取得します
      // ゴミが見つからなかった
      if(!this->recognizeTrash(m_tpos,m_tname)){
        //broadcastMsgToSrv("No trash detected");
        //broadcastMsgToSrv("Task finished");
        sleep(10);
      }
      // ゴミが見つかった trash detected
      else{
        //broadcastMsgToSrv("Please show which trash to take\n");
        sendMsg("VoiceReco_Service"," Please show me which object to take ");
        m_state = 7;
      }
    }
    break;
  }
  // wating to point
  case 7: {
    break;
  }
  // 物体認識開始 starting object recognition
  case 8: {
    //  m_tpos object direction on object
    SimObj *target = this->getObj(m_pointedObject.c_str());
    target->getPosition(m_tpos);
    //broadcastMsgToSrv("Ok I will take it\n");
    msg_ob = " I will take " + m_pointedObject ;
    sendMsg("VoiceReco_Service",msg_ob);
    // ゴミの方向に回転をはじめる
    m_time = rotateTowardObj(m_tpos, m_vel, evt.time());
    m_state = 9;
    break;
  }
  // ゴミの方向に回転をはじめる setting trash position for rotating
  case 9: {
    m_time = rotateTowardObj(m_tpos, m_vel, evt.time());
    m_state = 10;
    break;
  }
  // ゴミの方向に回転中 rotating to trash
  case 10: {
    // 回転終了
    if(evt.time() >= m_time){
      // 回転を止める
      m_my->setWheelVelocity(0.0, 0.0);
      //ゴミの位置まで移動をはじめる setting trash position for moving
      m_time = goToObj(m_tpos, m_vel*4, 25.0, evt.time());
      m_state = 11;
    }
    break;
  }
  // ゴミの位置まで移動中 moving to trash
  case 11: {
    // 移動終了
    if(evt.time() >= m_time){
      // 移動を止める
      m_my->setWheelVelocity(0.0, 0.0);
      // 関節の回転を始める setting arm for grasping
      double angR1 =m_my->getJointAngle("RARM_JOINT1")*180.0/(PI);
      double angR4 =m_my->getJointAngle("RARM_JOINT4")*180.0/(PI);
      double thetaR1 = -30.0-angR1;
      double thetaR4 = 0.0-angR4;
      if(thetaR1<0) m_my->setJointVelocity("RARM_JOINT1", -m_jvel, 0.0);
      else m_my->setJointVelocity("RARM_JOINT1", m_jvel, 0.0);
      if(thetaR4<0) m_my->setJointVelocity("RARM_JOINT4", -m_jvel, 0.0);
      else m_my->setJointVelocity("RARM_JOINT4", m_jvel, 0.0);
      m_time_RA1 = DEG2RAD(abs(thetaR1) )/ m_jvel + evt.time();
      m_time_RA4 = DEG2RAD(abs(thetaR4) )/ m_jvel + evt.time();
      // ゴミを取りに関節を曲げる状態に移行します
      m_state = 12;
    }
    break;
  }
  // 関節を回転中 rotating arm for grasping
  case 12: {
    // 関節回転終了
    if(evt.time() >= m_time_RA1) m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
    if(evt.time() >= m_time_RA4) m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
    if(evt.time() >= m_time_RA1 && evt.time() >= m_time_RA4){
      if(m_grasp) {
        //broadcastMsgToSrv("grasping the trash");
        // 関節の回転を始める setting arm for taking
        double angR4 =m_my->getJointAngle("RARM_JOINT4")*180.0/(PI);
        double thetaR4 = -90.0-angR4;
        if(thetaR4<0) m_my->setJointVelocity("RARM_JOINT4", -m_jvel, 0.0);
        else m_my->setJointVelocity("RARM_JOINT4", m_jvel, 0.0);
        m_time_RA4 = DEG2RAD(abs(thetaR4) )/ m_jvel + evt.time();
        // 関節を戻す状態に移行します
        m_state = 13;
      }
      else{
        // graspできない
        broadcastMsgToSrv("Unreachable");
      }
    }
    break;
  }
  // 関節を回転中 rotating arm for taking
  case 13: {
    // 関節回転終了
    if(evt.time() >= m_time_RA4){
        m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
        // 位置Aの方向に回転を開始します setting position a for rotating
        //broadcastMsgToSrv("Moving to the trashbox");
        sendMsg("VoiceReco_Service"," Now I will go to the trashboxes ");
        m_time = rotateTowardObj(pos_a, m_vel, evt.time());
        m_state = 14;
    }
    break;
  }
  // 位置Aの方向に回転 rotating to position a
  case 14: {
    // 回転終了
    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);
      // 位置Aに移動します setting position a for moving
      m_time = goToObj(pos_a, m_vel*4, 0.0, evt.time());
      m_state = 15;
    }
    break;
  }
  // 位置Aの位置まで移動中 movig to position a
  case 15: {
    // 移動終了
    if(evt.time() >= m_time){
     m_my->setWheelVelocity(0.0, 0.0);
     //broadcastMsgToSrv("Please tell me which trash box \n");
     sendMsg("VoiceReco_Service"," Please show me which trashbox to use ");
     m_state = 16;
    }
    break;
  }
  // waitig to point4
  case 16: {
    break;
  }
  // ゴミ箱認識 starting trash box recognitiong-g-0
  case 17: {
    //  m_tpos object direction on object
    SimObj *target_trash = this->getObj(m_pointedtrash.c_str());
    target_trash->getPosition(m_tpos);
    //broadcastMsgToSrv("Ok I will throw the trash in trash box \n");
    msg_trash = " Ok I will put "+ m_pointedObject+" in "+ m_pointedtrash +" trashbox ";
    sendMsg("VoiceReco_Service",msg_trash);
    // ゴミの方向に回転をはじめる setting position trash box for rotating
    m_time = rotateTowardObj(m_tpos, m_vel, evt.time());
    m_state = 18;
    break;
  }
  // ゴミ箱の方向に回転中 rotating to trash box
  case 18: {
    if(evt.time() >= m_time){
      // 回転を止める
      m_my->setWheelVelocity(0.0, 0.0);
      //ゴミの位置まで移動をはじめる setting trash position for moving
      m_time = goToObj(m_tpos, m_vel*4, 30.0, evt.time());
      m_state = 19;
    }
    break;
  }
  // ゴミを持ってゴミ箱に向かっている状態 moving to trash box
  case 19: {
    // ゴミ箱に到着
    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);
      // grasp中のパーツを取得します getting grasped tarts
      CParts *parts = m_my->getParts("RARM_LINK7");
      // releaseします
      parts->releaseObj();
      // ゴミが捨てられるまで少し待つ
      sleep(1);
      // 捨てたゴミをゴミ候補から削除 deleting grasped object from list
      std::vector<std::string>::iterator it;
      it = std::find(m_trashes.begin(), m_trashes.end(), m_pointedObject);
      m_trashes.erase(it);
      // grasp終了
      m_grasp = false;
      m_state = 1;
    }
    break;
  }
  }
  return 0.01;
}

void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
  std::string sender = evt.getSender();

  // 送信者がゴミ認識サービスの場合
  char *all_msg = (char*)evt.getMsg();
  std::string msg;
  msg= evt.getMsg();

  if(msg == "go" && m_state ==20)
  {
        m_state = 0;
  }
  else if(msg == "take" && m_state == 7)
  {
        printf("Kinect is started on object\n");
        m_pointedObject = getPointedObjectName(m_avatar);
        m_state = 8;
  }
  else if(msg == "put" && m_state == 16)
  {
        printf("Kinect is started on trash\n");
        m_pointedtrash = getPointedTrashName(m_avatar);
        m_state = 17;
  }
  else
  {
        printf("Message is not accepted\n");
  }
}


std::string RobotController::getPointedObjectName(std::string entName)
{
  // 発話者の名前からSimObjを取得します
  SimObj *tobj = getObj(entName.c_str());

  // メッセージ送信者の左肘関節の位置を取得します
  Vector3d jpos;
  if(!tobj->getJointPosition(jpos, "RARM_JOINT4")) {
    LOG_ERR(("failed to get joint position"));
    return "";
  }
  // メッセージ送信者の左肘から左手首をつなぐベクトルを取得します
  Vector3d jvec;
  if(!tobj->getPointingVector(jvec, "RARM_JOINT4", "RARM_JOINT7")) {
    LOG_ERR(("failed to get pointing vector"));
    return "";
  }

  double distance = 0.0;
  std::string objName = "";

  // 全ゴミオブジェクトでループします
  int trashSize = m_trashes.size();
  for(int i = 0; i < trashSize; i++) {

  // エンティティの位置を取得します
  SimObj *obj = getObj(m_trashes[i].c_str());
  Vector3d objVec;
  obj->getPosition(objVec);

  // エンティティと左肘関節を結ぶベクトルを作成します
  objVec -= jpos;

  // cos角度が不の場合（指差した方向と反対側にある場合)は対象から外します
  double cos = jvec.angle(objVec);
  //if(cos < 0)
  //  continue;

  // 指差した方向ベクトルまでの最短距離の計算
  double theta = acos(cos);
  double tmp_distance = sin(theta) * objVec.length();

  // 最小距離の場合は名前、距離を保存しておく
  if(tmp_distance < distance || distance == 0.0){
    distance = tmp_distance;
    objName = obj->name();
    }
  }
  // エンティティでループして最も近いオブジェクトの名前を取得する
  return objName;
}

std::string RobotController::getPointedTrashName(std::string entName)
{
  // 発話者の名前からSimObjを取得します
  SimObj *tobj = getObj(entName.c_str());

  // メッセージ送信者の左肘関節の位置を取得します
  Vector3d jpos;
  if(!tobj->getJointPosition(jpos, "RARM_JOINT4")) {
    LOG_ERR(("failed to get joint position"));
    return "";
  }
  // メッセージ送信者の左肘から左手首をつなぐベクトルを取得します
  Vector3d jvec;
  if(!tobj->getPointingVector(jvec, "RARM_JOINT4", "RARM_JOINT7")) {
    LOG_ERR(("failed to get pointing vector"));
    return "";
  }

  double distance = 0.0;
  std::string objName = "";

  // 全ゴミオブジェクトでループします
  int trashboxSize = m_trashboxs.size();
  for(int i = 0; i < trashboxSize; i++) {

  // エンティティの位置を取得します
  SimObj *obj = getObj(m_trashboxs[i].c_str());
  Vector3d objVec;
  obj->getPosition(objVec);

  // エンティティと左肘関節を結ぶベクトルを作成します
  objVec -= jpos;

  // cos角度が不の場合（指差した方向と反対側にある場合)は対象から外します
  double cos = jvec.angle(objVec);
  //if(cos < 0)
  //  continue;

  // 指差した方向ベクトルまでの最短距離の計算
  double theta = acos(cos);
  double tmp_distance = sin(theta) * objVec.length();

  // 最小距離の場合は名前、距離を保存しておく
  if(tmp_distance < distance || distance == 0.0){
    distance = tmp_distance;
    objName = obj->name();
    }
  }
  // エンティティでループして最も近いオブジェクトの名前を取得する
  return objName;
}

bool RobotController::recognizeTrash(Vector3d &pos, std::string &name)
{
  // 候補のゴミが無い場合
  if(m_trashes.empty()){
    return false;
  }

  // ここでは乱数を使ってゴミを決定します
  int trashNum = rand() % m_trashes.size();

  // ゴミの名前と位置を取得します
  name = m_trashes[trashNum];
  SimObj *trash = getObj(name.c_str());

  // ゴミの位置取得
  trash->getPosition(pos);
  return true;
}

void RobotController::onCollision(CollisionEvent &evt)
{
  if (m_grasp == false){
    typedef CollisionEvent::WithC C;
    //触れたエンティティの名前を得ます
    const std::vector<std::string> & with = evt.getWith();
    // 衝突した自分のパーツを得ます
    const std::vector<std::string> & mparts = evt.getMyParts();
    //　衝突したエンティティでループします
    for(int i = 0; i < with.size(); i++){
      //右手に衝突した場合
      if(mparts[i] == "RARM_LINK7"){
        //自分を取得
        SimObj *my = getObj(myname());
        //自分の手のパーツを得ます
        CParts * parts = my->getParts("RARM_LINK7");
        if(parts->graspObj(with[i])){
          m_grasp = true;
        }
      }
    }
  }
}

double RobotController::rotateTowardObj(Vector3d pos, double velocity, double now)
{
  // 自分の位置の取得
  Vector3d myPos;
  //m_my->getPosition(myPos);
  m_my->getPartsPosition(myPos,"RARM_LINK2");

  // 自分の位置からターゲットを結ぶベクトル
  Vector3d tmpp = pos;
  tmpp -= myPos;

  // y方向は考えない
  tmpp.y(0);

  // 自分の回転を得る
  Rotation myRot;
  m_my->getRotation(myRot);

  // エンティティの初期方向
  Vector3d iniVec(0.0, 0.0, 1.0);

  // y軸の回転角度を得る(x,z方向の回転は無いと仮定)
  double qw = myRot.qw();
  double qy = myRot.qy();

  double theta = 2*acos(fabs(qw));

  if(qw*qy < 0)
    theta = -1*theta;

  // z方向からの角度
  double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
  double targetAngle = acos(tmp);

  // 方向
  if(tmpp.x() > 0) targetAngle = -1*targetAngle;
  targetAngle += theta;

  if(targetAngle == 0.0){
    return 0.0;
  }
  else {
    // 回転すべき円周距離
    double distance = m_distance*PI*fabs(targetAngle)/(2*PI);

    // 車輪の半径から移動速度を得る
    double vel = m_radius*velocity;

    // 回転時間(u秒)
    double time = distance / vel;

    // 車輪回転開始
    if(targetAngle > 0.0){
      m_my->setWheelVelocity(velocity, -velocity);
    }
    else{
      m_my->setWheelVelocity(-velocity, velocity);
    }

    return now + time;
  }
}

// object まで移動
double RobotController::goToObj(Vector3d pos, double velocity, double range, double now)
{
  Vector3d myPos;
  //m_my->getPosition(myPos);
  m_my->getPartsPosition(myPos,"RARM_LINK2");

  // 自分の位置からターゲットを結ぶベクトル
  pos -= myPos;

  // y方向は考えない
  pos.y(0);

  // 距離計算
  double distance = pos.length() - range;

  // 車輪の半径から移動速度を得る
  double vel = m_radius*velocity;

  // 移動開始
  m_my->setWheelVelocity(velocity, velocity);

  // 到着時間取得
  double time = distance / vel;

  return now + time;
}

//自身のインスタンスをSIGVerseに返します
extern "C" Controller * createController() {
  return new RobotController;
}
