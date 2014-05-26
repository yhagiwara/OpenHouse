#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h" 
#include <unistd.h>
 
  
class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);  
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt); 

private:
  SimObj *m_my;
  std::vector<std::string> m_entities;
  
  // ゴミ箱のサイズ(この範囲でreleaseしなければゴミを捨てられない)
  double tboxSize_x, tboxSize_z;

  // ゴミが入ったとされる高さ方向の範囲(y方向)
  double tboxMin_y, tboxMax_y;

  // 審判サービス
  BaseService *m_ref;
  
  bool   colState;      // 衝突中かどうか
  double retValue;
  std::string roboName;
};  
  
void MyController::onInit(InitEvent &evt) {  
  m_my = getObj(myname());
  getAllEntities(m_entities);
  m_ref = NULL;
  retValue = 0.5;
  roboName = "robot_000";

  colState = false;

  // ゴミ箱
  tboxSize_x  = 20.0;
  tboxSize_z  = 40.5; 
  tboxMin_y    = 40.0;
  tboxMax_y    = 1000.0;
}  
  
double MyController::onAction(ActionEvent &evt) 
{ 
  // サービスが使用可能か定期的にチェックする  
  bool available = checkService("RobocupReferee");  

  if(!available && m_ref != NULL) m_ref = NULL;

  // 使用可能  
  else if(available && m_ref == NULL){  
    // サービスに接続  
    m_ref = connectToService("RobocupReferee");  
  }  
 
  // 自分の位置取得
  Vector3d myPos;
  m_my->getPosition(myPos);

  // 衝突中の場合,衝突が継続しているかチェック
  if(colState){
    CParts *parts = m_my->getMainParts();
    bool state = parts->getCollisionState();
    
    // 衝突していない状態に戻す
    if(!state) colState = false;
  }
  
  int entSize = m_entities.size();
  for(int i = 0; i < entSize; i++){

    // ロボットまたはゴミ箱の場合は除く
    if(m_entities[i] == "robot_000"  ||
       m_entities[i] == "recycle" ||
       m_entities[i] == "burnable" ||
       m_entities[i] == "unburnable"){
      continue;
    }
    // エンティティ取得
    SimObj *ent = getObj(m_entities[i].c_str());

    // 位置取得
    Vector3d tpos;
    ent->getPosition(tpos);

    // ゴミ箱からゴミを結ぶベクトル
    Vector3d vec(tpos.x()-myPos.x(), tpos.y()-myPos.y(), tpos.z()-myPos.z());
    
    // ゴミがゴミ箱の中に入ったかどうか判定
    if(abs(vec.x()) < tboxSize_x/2.0 &&
       abs(vec.z()) < tboxSize_z/2.0 &&
       tpos.y() < tboxMax_y     &&
       tpos.y() > tboxMin_y     ){

      // ゴミがリリースされているか確認
      if(!ent->getIsGrasped()){

	// ゴミを捨てる
	tpos.y(tpos.y() /2);
	tpos.x(myPos.x());
	tpos.z(myPos.z());
	ent->setAxisAndAngle(1.0, 0.0, 0.0, 0.0);
	ent->setAxisAndAngle(1.0, 0.0, 0.0, 0.0);
	ent->setPosition(tpos);
	ent->setPosition(tpos);
	ent->setPosition(tpos);
	usleep(500000);
	tpos.y(0.0);
	ent->setPosition(tpos);
	ent->setPosition(tpos);
	ent->setPosition(tpos);

	std::string msg;
	// ゴミが所定のゴミ箱に捨てられているかチェック
	// リサイクル
	if(strcmp(myname(), "recycle") == 0){
	  // 空のペットボトルのみ点が入る
	  if(strcmp(ent->name(), "petbottle") == 0 ||
	     strcmp(ent->name(), "petbottle_2") == 0 ||
	     strcmp(ent->name(), "petbottle_4") == 0 ||
	     strcmp(ent->name(), "mayonaise_1") == 0 ) {
	    msg = "RobocupReferee/Clean up succeeded" "/1000";
	  }
	  else{
	    msg = "RobocupReferee/Clean up failed" "/-600";
	  }
	}
	// 燃えるゴミ
	else if(strcmp(myname(), "burnable") == 0){
	  // 燃えるゴミに入れるべきものは無い
	  msg = "RobocupReferee/Clean up failed" "/-600";
	}
	// 缶瓶
	else if(strcmp(myname(), "unburnable") == 0){
	  if(strcmp(ent->name(), "can_0") == 0 ||
	     strcmp(ent->name(), "can_1") == 0 ||
	     strcmp(ent->name(), "can") == 0 ||
	     strcmp(ent->name(), "can_3") == 0) {
	    msg = "RobocupReferee/Clean up succeeded" "/1000";
	  }
	  else {
	    msg = "RobocupReferee/Clean up succeeded" "/-600";
	  }
	}

	if(m_ref != NULL) {
	  m_ref->sendMsgToSrv(msg.c_str());
	}
	LOG_MSG((msg.c_str()));
      }
    }
  }

  return retValue;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt) {  
}  

void MyController::onCollision(CollisionEvent &evt) { 
  // 衝突していない状態のときのみ衝突をチェック
  if(!colState) {
    const std::vector<std::string> & wname= evt.getWith();
    int csize = wname.size();
    for(int i = 0; i < csize; i++){
      // robotと衝突
      if(wname[i] == roboName){
	colState = true;
	std::string msg = "RobocupReferee/Collision with [" + std::string(myname()) + "]" "/-100";
	if(m_ref != NULL){
	  m_ref->sendMsgToSrv(msg.c_str());
	}
	else{
	  LOG_MSG((msg.c_str()));
	}
      }
    }
  }
}
  
extern "C" Controller * createController() {  
  return new MyController;  
}  

