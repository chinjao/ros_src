#include "include/speech.h"

extern void baxterarm_up(speech_msgs::Extract ext_msg,string name,string admin);
extern void robot_browser(string action,int admin);
extern void extract(speech_msgs::SpeechConstPtr speech,int okao_id,string name);


struct aura{
  long long trackingID;
  int positive_point;
  MSGPACK_DEFINE(trackingID,positive_point);
}req;


class Place
{
public:
  string name;
  double x;
  double y;
};

class Situation
{
public:
  int okao_id;
  Place place;
  ros::Time time;
  vector<int> people;
};

class WordProp
{
public:
  int freq;
  int num;
  vector<string> part;
  vector<Situation> situations;
};

//map< int, map< string, WordProp > > user_words;
int conv = 0;

map< string, WordProp > words_wp;
map< int, humans_msgs::Human > human_last;
map<string , float > mood;
map<int, map < string ,float> > window_mood;
vector<int> time_window;
map<int,string> human_time;
int window_num = 0;

int time_number = 0;
string username;

class SpeechRecog{
private:
  ros::NodeHandle nh;
  ros::ServiceServer word_srv;
  //ros::Publisher recog_pub_;
  //boost::mutex mutex_;
  ros::Subscriber speech_sub;
  ros::Subscriber emotion_sub;
  ros::Subscriber okao_sub;
  ros::Publisher conv_pub;
  ros::Publisher sp_pub;

public:
  SpeechRecog() 
  {

    speech_sub 
      = nh.subscribe("emotion_info", 1, 
		     &SpeechRecog::speechCallback, this);

    okao_sub
      = nh.subscribe("/humans/RecogInfo", 1,
		     &SpeechRecog::okaoCallback, this);


    word_srv 
      = nh.advertiseService("word_search",
			    &SpeechRecog::wordSearchCallback, this);
    //recog_pub_ = 
    //  nh.advertise<humans_msgs::Humans>("/humans/RecogInfo", 1);

    //ファイルからメモリにこれまでの単語情報を書き込む機能

    conv_pub = nh.advertise<std_msgs::String>("req_conv", 1000);
    sp_pub = nh.advertise<speech_msgs::Emotion>("emo_and_name",1000);

  }
  ~SpeechRecog()
  {
    //メモリからファイルにこれまでの単語情報を書き込む機能
    cout << "file write" << endl;
    stringstream ss;
    map< string, WordProp >::iterator it_wp = words_wp.begin();
    while( it_wp != words_wp.end() )
      {

	ss << "word: " << it_wp->first << ", freq: " << it_wp->second.freq 
	   << ", num: "<< it_wp->second.num; 

	
	//stringstream sit;
	for(int w_id = 0; w_id < it_wp->second.situations.size() ; ++w_id)
	  {
	    ss << ", situation[ " << w_id <<" ]: " <<  it_wp->second.situations[ w_id ].okao_id; 
	  }
	
	ss << endl;

	++it_wp; 
      }

    std::ofstream ofs("/home/yhirai/catkin_ws/src/okao_client/src/people/words.txt");//,std::ios::out | std::ios::app );
    if(ofs.fail())
      {  // if(!fout)でもよい。
        cout << "file not open" << endl;
        return;
      }
    ofs << ss.str() <<endl;
    cout << "write data:" << ss.str() <<endl;


    //
    words_wp.clear();
  }
 
  void okaoCallback(
		    const humans_msgs::HumansConstPtr& okao
		    )
  {
    //cout << "okao callback people num: "<< okao->human.size() << endl;
    //bool master_is_looking = false;
    if( okao->human.size() )
      {
	cout<< "add okao size: " << okao->human.size()<<endl; 
	//過去の人物情報を一旦クリア
	human_last.clear();
	for(int h_i = 0 ; h_i < okao->human.size() ; ++h_i)
	  {
	    human_last[ h_i ] = okao->human[ h_i ];
	    
	    if( okao->human[ h_i ].max_okao_id == MASTER )
	      {
		cout << "RECOG OK! Master: " << okao->human[ h_i ].max_okao_id << endl; 
	      }
	    
	  }
      }

  }

  void speechCallback(
		      const speech_msgs::SpeechConstPtr& speech
		      )
  {
    cout << "speech callback" << endl;
    
    //1.開始
    //2.okao内のトラッキングIDと結びついた名前（OKAO_ID）を調べる
    //3.もし、主人の名前があったら、4へ。なければ1へ（ループ）
    //4.speech内のトラッキングIDを取得
    //5.トラッキングIDとOKAO_IDを照合する
    //6.OKAO_IDに発言を記録する
    //8.主人以外の人物の記録が終わったら、1へ。終わってなければ6へ。
  
    //人物を検出しているかどうか
    //   cout << "human size: "<< human_last.size() <<endl;

    /* if(human_last.size())
       {*/
	bool master_is_looking = false;
	map<long long, int> tracking_to_okao;
	//発見した人物を記録し、主人を探す
	for(int i = 0; i<human_last.size(); ++i)
	  {
	    tracking_to_okao[ human_last[i].body.tracking_id ] 
	      = human_last[i].max_okao_id;
      	    cout << "now looking okao_id: " << human_last[i].max_okao_id << endl; 
	    if( human_last[i].max_okao_id == MASTER )
	      master_is_looking = true;
	  }

	//主人がいるなら記録開始
	//	if( master_is_looking )
	//{
	speech_msgs::Speech sp_msg;
	float point_p = 0;

	if(time_number == 0)
	  username = "Wakimoto";
	else if(time_number == 1)
	  username = "Hirai";
	else if(time_number == 2)
	  username = "Wakimoto";
	else if(time_number == 3)
	  username = "Hirai";
	else if(time_number == 4)
	  username = "Hirai";
	else if(time_number == 5)
	  username = "Wakimoto";
	else if(time_number == 6)
	  username = "Wakimoto";
	else if(time_number == 7)
	  username = "Hirai";
	else if(time_number == 8)
	  username = "Wakimoto";
	else if(time_number == 9)
	  username = "Hirai";
	/*	else if(time_number == 10)
		username = "Hirai";*/
	else 
	  username = "Hirai";
			

	cout<< "now looking master!" <<endl;
	int speech_okao_id;
	//string to long long
	long long speech_tracking_id = boost::lexical_cast<
	  long long>(speech->TrackingID); 
	cout << "now speech tracking_id: " << speech_tracking_id << endl;


       	map< long long, int >::iterator tracking_id_find
	  = tracking_to_okao.find( speech_tracking_id );
	cout << "Now Speech Emotion:" <<  speech->emotion << endl;
	speech_okao_id = tracking_id_find->second;
	cout << "Now Speech Okao_ID:" << speech_okao_id << endl;
	speech_pub(speech,speech_okao_id);
	browser(speech,speech_okao_id);
	time_number++;
	


  }

  void conversation(string sentence){
    //  cout << "きてます" << speech->sentence << endl;

    std_msgs::String req_conv_msg;
    req_conv_msg.data = sentence;
    conv_pub.publish(req_conv_msg);
    // cout << req_conv_msg.data << endl;
    std_msgs::StringConstPtr res_conv_msg = ros::topic::waitForMessage<std_msgs::String>("res_conv");
    robot_browser(res_conv_msg->data,1);
    //   cout << res_conv_msg->data << endl;
    
  }

  string TIDtoName(int okao_id){
    string name;
    for(int h_i = 0; h_i < human_last.size() ; ++h_i)
      {
	if( okao_id == human_last[ h_i ].max_okao_id 
	    && human_last[ h_i ].face.persons.size() )
	  {
	    cout<<"person size: " << human_last[ h_i ].face.persons.size() <<endl;
	    name = human_last[ h_i ].face.persons[0].name;
	    break;
	  }
	else
	  {
	    name = "Unknown";
	  }
      }
    return name;
  }

  bool wordcheck(string word,string part){
    if(part == "助詞" || part == "助動詞" || part == "記号" || word == "<->" || word == "")
      {
	cout << "part: " << part << ", word: "<< word << endl;
	return 0;
      }
    else
      return 1;
  }
  
  void browser(speech_msgs::SpeechConstPtr speech,int okao_id){
    if(speech->sentence == "<->")
      return;
    cout << "browser" <<endl;
    zmq::context_t context(1);
    zmq::socket_t sender(context,ZMQ_PUSH);
    sender.connect("tcp://133.19.23.224:8081");
    map<string, string> dict;
    cout << "in" << endl;
    dict["senderName"] = "speechRecognition";
    string name;
    name = username;//TIDtoName(okao_id);
    cout<<"name:"<<name <<endl;
    dict["name"] = name;
    //dict["time"] = speech->time;
    string emo_hi;
    double si_pos,si_neg,si_nor;
    si_pos = round(speech->positive * 100);
    si_pos /= 100;
    si_neg = round(speech->negative * 100);
    si_neg /= 100;
    si_nor = round(speech->normal * 100);
    si_nor /= 100;
    char posi[70],nega[70],norm[70];
    sprintf(posi,"%.2f",si_pos);
    sprintf(nega,"%.2f",si_neg);
    sprintf(norm,"%.2f",si_nor);
    emo_hi = "observation:" + speech->emotion;
    dict["place"] = emo_hi;
    cout <<"bayesmae" << endl;

    speech_msgs::BayesianConstPtr bayes_msg = ros::topic::waitForMessage<speech_msgs::Bayesian>("bayes_result");
    cout << "result receive" << endl;
    string emotion_rec = bayes_msg->emotion;
    string visual = "emotion:" + emotion_rec;
    dict["speechRec"] = visual;
    
    if(emotion_rec == "happy"){
      //cout << "positive point: " << "positive" << endl;
      dict["mood"] = "255";
	//	dict["speechRec"] = "positive";
    }
    else if(emotion_rec == "sad"){
      dict["mood"] = "-255";
      //   dict["speechRec"] = "negative";
    }
    else if(emotion_rec == "quiet"){
      dict["mood"] = "0";
      //    dict["speechRec"] = "normal";
    }
    else{
      dict["mood"] = "0";
      //	dict["speechRec"] = "error";
    }

    msgpack::sbuffer buffer;
    msgpack::pack(buffer,dict);
    zmq::message_t message(buffer.size());
    memcpy(message.data(),buffer.data(),buffer.size());
    sender.send(message);

    cout << "sender message" << endl;
  }



  void speech_pub(speech_msgs::SpeechConstPtr speech,int okao_id){
    speech_msgs::Emotion emo_msg2;
    string name = username;//TIDtoName(okao_id);
    emo_msg2.name = name;
    emo_msg2.emotion = speech->emotion;
    emo_msg2.positive = speech->positive;
    emo_msg2.negative = speech->negative;
    emo_msg2.normal = speech->normal;
    cout << "pub_now" << endl;
    //    for(int io = 0; io < 100 ;io++)
    sp_pub.publish(emo_msg2);
    
  }


  
  void zmq_positive(long long tracking_ID,string emotion){
    zmq::context_t context(1);
    zmq::socket_t sender(context,ZMQ_PUSH);
    sender.connect("tcp://133.19.23.205:5733"); //ip address
    req.trackingID = tracking_ID;
    int positive_point = 0;
    if(emotion == "positive")
      positive_point = 5;
    else if(emotion == "negative")
      positive_point = -5;
    req.positive_point = positive_point;
    msgpack::sbuffer sbuf;
    msgpack::pack(&sbuf,req);
    zmq::message_t message_send(sbuf.size());
    memcpy(message_send.data(),sbuf.data(),sbuf.size());
    sender.send(message_send);
    cout << "okzmq_pos" << endl;
  }


  //検索用関数
  //今はもっともその単語を発話した人物と、その回数のみを出力
  //今後は発話された場所や、時間の情報、発話したすべての人物
  bool wordSearchCallback(speech_msgs::SpeechSrv::Request &req,
			  speech_msgs::SpeechSrv::Response &res)
  {
    cout<<"now search word: "<< req.word << endl;
  
    map< string, WordProp >::iterator it_s
      = words_wp.find( req.word );

    if( it_s != words_wp.end() )
      {
	//もし単語が見つかったら、その単語を発した人全員を返す？
	//いまは最大値でいいんじゃない？
	map< int,int > hist;
	vector<Situation>::iterator it_name = it_s->second.situations.begin();
	while(it_name != it_s->second.situations.end() )
	  {
	    ++hist[it_name->okao_id];
	    ++it_name;
	  }

	map< int,int >::iterator it_hist = hist.begin();

	int max_id = it_hist->first;
	++it_hist;
	while( it_hist != hist.end() )
	  {
	    if( hist[ max_id ] < hist[ it_hist->first ] )
	      {
		max_id = it_hist->first;
	      }
	    ++it_hist;
	  }
	res.okao_id = max_id;
	res.freq = hist[ max_id ];
	return true;
      }

    /*
      while( it_s != user_words.end() )
      {    
      //単語があるかどうかを調べる
      map< string, WordProp >::iterator it_w 
      = it_s->second.find( req.word );

      if( it_w != it_s->second.end() )
      {
      //単語があるなら、そのときの頻度などを返す

      return true;
      }

      ++it_s;
      }
    */

    return false;
  }

};
  
void check(){
  int before_sixty = -1;
  int before_thirty = -1;
  while(1){
    int i;
    int thirty = -1;
    int sixty = -1;
    sleep(2);
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);
    int all = ((pnow->tm_hour*60+pnow->tm_min)*60)+pnow->tm_sec;
    //    cout << all << endl;
    if(window_mood.size() != 0){
      for(i = window_num-1 ; i >= 0 ; i--){
	int dif = all - time_window[i];
	//cout << dif << endl;
	//cout << time_window[i] << endl;
	if (dif > 30 && thirty == -1){
	  thirty = i;
	  // cout << "ok:" << thirty << endl;
	}
	if(dif > 120 && sixty == -1){
	  sixty = i;
	  //  cout << "ok2:" << sixty << endl;

	}
      }
      //if(){
	int aa;
	if(sixty == -1)
	  aa = 0;
	else if(before_sixty != -1 && sixty == before_sixty){
	  aa = before_sixty;
	}
	else 
	  aa = sixty;
	//	int l = thirty;
       	for(int l = window_num-1; l >= aa ; l--){
	  double sub;
	  if( window_mood[time_window[l]][human_time[time_window[l]]] == 0)
	    break;
	  sub = all-time_window[l];
	  
	  window_mood[time_window[l]][human_time[time_window[l]]] *= pow(0.1,(sub/120));
	  //  cout << pow(2,0.05) << endl;
	  cout <<  window_mood[time_window[l]][human_time[time_window[l]]] << endl;
	}
	//	before_thirty = thirty;
	//cout << "in" << endl;
	//}
      if(sixty >= 0 && sixty != before_sixty){
	int l = sixty;
	//for(int l = sixty; l > 0 ; l--)
	window_mood[time_window[l]][human_time[time_window[l]]] = 0;
	before_sixty = sixty;
	
	cout << "in2" << endl;
      }
    }
  }
}

  int main(int argc, char** argv){
    thread thr_check(&check);
    ros::init(argc, argv, "speech_recog");
    SpeechRecog SRObject;
    ros::MultiThreadedSpinner spinner(3); // Use 3 threads
    spinner.spin(); 
    //ros::spin();
    return 0;
  }
