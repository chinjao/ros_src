#include "include/speech.h"

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
map<int, map<int , map< int , map < string ,float> > > > window_mood;
time_t now = time(NULL);

class SpeechRecog{
private:
  ros::NodeHandle nh;
  ros::ServiceServer word_srv;
  //ros::Publisher recog_pub_;
  //boost::mutex mutex_;
  ros::Subscriber speech_sub;
  ros::Subscriber okao_sub;
  ros::Publisher conv_pub;

public:
  SpeechRecog() 
  {

    speech_sub 
      = nh.subscribe("mecab_res", 1, 
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
    cout << "human size: "<< human_last.size() <<endl;

    if(human_last.size())
      {
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

	cout<< "now looking master!" <<endl;
	int speech_okao_id;
	//string to long long
	long long speech_tracking_id = boost::lexical_cast<
	  long long>(speech->TrackingID); 
	cout << "now speech tracking_id: " << speech_tracking_id << endl;

	map< long long, int >::iterator tracking_id_find
	  = tracking_to_okao.find( speech_tracking_id );
	speech_okao_id = tracking_id_find->second;
	sp_msg.okao_id = speech_okao_id;
	if(speech->positive_point == 1){
	  point_p = 40;
	}
	else if(speech->positive_point == -1){
	  point_p = -40;
	}
	  //speech->positive_point = -20;
	cout << "point: " << speech->positive_point << endl;
	
	if(mood.count(TIDtoName(speech_okao_id)) == 0){
	  mood[TIDtoName(speech_okao_id)] = point_p;
	}
	else{
	  float point = mood[TIDtoName(speech_okao_id)];
	  point += point_p;
	  mood[TIDtoName(speech_okao_id)] = point;
	}
	window_mood[speech->hour][speech->minute][speech->second][TIDtoName(speech_okao_id)] = point_p;
	
	  
	
	zmq_positive(speech_tracking_id,mood[TIDtoName(speech_okao_id)]);
	browser(speech,speech_okao_id);
	if(speech->sentence == "おしゃべりしよう" || speech->sentence == "お話ししよう"){
	  if(conv == 0){
	    conv = 1;
	    conversation("start");
	    //cout << "お話開始" << endl; 
	    //robot_browser("いいですよ",1);
	  }
	}
	else if(speech->sentence == "おしゃべり終わり" || speech->sentence == "おしゃべり終了"){
	  if(conv == 1){
	    conv = 0;
	    conversation("end");
	    //robot_browser("終了しますね",1);
	  }
	}
	else if(conv == 1){
	  conversation(speech->sentence);
	}
	else
	  extract(speech,speech_okao_id);
	cout << "now speech okao_id: " << speech_okao_id << endl;
	cout << "all words: "<< speech->sentence <<endl;
	for( int w_id = 0 ; w_id < speech->word_num ; ++w_id )
	  {
	    if(wordcheck(speech->words[w_id].word,speech->words[w_id].part[0])){
	      WordProp wp_tmp;
	      wp_tmp.freq = 0;
	      wp_tmp.num = 0;  
	      map< string, WordProp >::iterator it_words_wp
		= words_wp.find( speech->words[ w_id ].word );
	      if( it_words_wp != words_wp.end() )
		{
		  wp_tmp = it_words_wp->second;
		}
		  
	      ++wp_tmp.freq;
	      wp_tmp.num = speech->word_num;
	      Situation st_tmp;
	      st_tmp.okao_id = speech_okao_id;

	
	      sp_msg.words.push_back( speech->words[w_id] );

	      //すべての人物をpush_backする
	      map< long long, int >::iterator it_okao 
		= tracking_to_okao.begin(); 
	      while(it_okao != tracking_to_okao.end() )
		{
		  st_tmp.people.push_back( it_okao->second ); 
		  ++it_okao;
		}
	      wp_tmp.situations.push_back( st_tmp );
		  
	      words_wp[ speech->words[ w_id ].word ] = wp_tmp; 
	    }
	  }
	//}
      }
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
 
    dict["senderName"] = "speechRecognition";
    string name;
    name = TIDtoName(okao_id);
    cout<<"name:"<<name <<endl;
    dict["name"] = name;
    dict["time"] = speech->time;
    dict["place"] = "pioneer";
    if(mood.count(name) != 0){
      int value = roundf(mood[name]);
      if(value < 255 && value > -255){
	cout << "positive point: " << value << endl;
	stringstream ss;
	ss << value;
	string str = ss.str();
	dict["mood"] = str;
      }
      else if(value > 255){
	dict["mood"] = "255";
      }
      else if(value < -255){
	dict["mood"] = "-255";
      }
    }
    int size_s = speech->sentence.size();
    //if(size_s <= 33){
    msgpack::sbuffer buffer;
    //   dict["speechRec"] = "abcdefghijklmnopqrstuvwxyz123456";
    dict["speechRec"] = speech->sentence;
    msgpack::pack(buffer,dict);
    zmq::message_t message(buffer.size());
    memcpy(message.data(),buffer.data(),buffer.size());
    sender.send(message);
  }
  
  void extract(speech_msgs::SpeechConstPtr speech,int okao_id){
    int i = 0;
    int d = 0;
    int no = 0;
    int b = 0;
    //    cout << "extract_start" << endl;
    speech_msgs::Extract ext_msg;
    ext_msg.com_jud = 0;
    for(int i = 0 ; i < speech->word_num ; ){
	if(speech->words[i].part[0] == "名詞" || speech->words[i].part[0] == "感動詞"){
	  cout << "extract now" << endl;
	  if(speech->words[i].part[0] == "名詞"){
	    struct stat st;
	    string textfile;
	    int ret;
	    const char* text;
	    if(speech->words[i].word == "バイバイ" || speech->words[i].word == "ばいばい" || speech->words[i].word == "さようなら"){
	      ext_msg.com = "bye";
	      ext_msg.com_jud = 1;
	    }
	    ext_msg.noun.push_back(speech->words[i].word);
	    d++;
	  }
	}
	else if(speech->words[i].part[0] == "動詞"){
	  cout << "verb now" << endl;
	  if(speech->words[i].word == "な"){
	    no = 1;
	  }
	  else{
	    if(speech->words[i].part[2] == "未然形"){
	      no = 1;
	    }
	    else{
	      if(speech->words[i].part[3] == "する"){
		for(b = 1; i-b != 0 ;b++){
		  if(speech->words[i-b].part[0] == "名詞"){
		    ext_msg.com = speech->words[i-b].word;
		    ext_msg.com_jud = 1;
		    //verb_pub.publish(verb_msg);
		    break;
		  }
		}
	      }
	      else{
		ext_msg.verb.push_back(speech->words[i].part[3]);
		//verb_pub.publish(verb_msg);
	      }
	    }
	  }
	}
	if(no != 1){
	  ext_msg.noun_num = d;
	}
	i++;
      }
	  cout << ext_msg.noun_num << endl;
	  if(ext_msg.verb.size() != 0 || ext_msg.com.size() != 0)
	    command(ext_msg,okao_id);
	
  }
  

  void command(speech_msgs::Extract ext_msg,int okao_id){
    int c;
    map<std::string,std::string> json_map,json_map2;
    value root,root2,root3;
    std::ifstream ifs("/home/yhirai/catkin_ws/src/noun/src/rank.txt");
    string rank[5];
    int i = 0;
    std_msgs::String command_msg;
    std_msgs::String chat_msg;
    ros::Publisher command_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    string aa;
    string bb = "なし";
    int robot = 0;
    int sub_robot = 0;
    int ok = 0;
    int chat_ok = 0;
    string com;
    string admin = "admin";
    string aa2;
    string name;
    name = TIDtoName(okao_id);
    if(ext_msg.verb.size() == 0)
      return;
    cout << "command now" << endl;
    if(ext_msg.com_jud == 1){
    }
    else{
      aa2 = ext_msg.verb[0];
      root2 = value();
      c = 0;
      ifstream stream2("/home/yhirai/catkin_ws/src/verb/src/verb_admin.json");
      stream2 >> root2;
      //   cout << com << endl;
      const picojson::value::object& obj2 = root2.get<picojson::object>();
      for (picojson::value::object::const_iterator i2 = obj2.begin(); i2 != obj2.end(); ++i2) {
	json_map2.insert( make_pair(i2->first,i2->second.to_str() ) );
      } 
      c = json_map2.count(aa2);
      if(c > 0){
	object image2 = root2.get<object>()[aa2].get<object>();
	// cout << "command=" << image2["command"].get<string>() << endl;
	com = image2["command"].get<string>();
	admin = image2["admin"].get<string>();
      }
      if(com == "move"){
	move_command(ext_msg,name,admin);
      }
      else if(com == "up"){
	baxterarm_up(ext_msg,name,admin);	
      }
    }
  }
    

    void zmq_positive(long long tracking_ID,int positive_point){
    zmq::context_t context(1);
    zmq::socket_t sender(context,ZMQ_PUSH);
    sender.connect("tcp://133.19.23.205:5733");
    req.trackingID = tracking_ID;
    req.positive_point = positive_point;
    msgpack::sbuffer sbuf;
    msgpack::pack(&sbuf,req);
    zmq::message_t message_send(sbuf.size());
    memcpy(message_send.data(),sbuf.data(),sbuf.size());
    sender.send(message_send);
    cout << "okzmq_pos" << endl;
  }


  void move_command(speech_msgs::Extract ext_msg,string name,string admin){
    int c;
    map<std::string,std::string> json_map;
    value root;
    int i = 0;
    std_msgs::String command_msg;
    ros::Publisher command_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    string aa;
    string com;
    string aa2;
    int chat_ok;
    string name_com;
    if(admin != name){
      cout << name << endl;
      robot_browser("no",0);
      cout << "あなたじゃだめですよ" << endl;
      return;
    }
    chat_ok = 1;
    for(i = 0; i < ext_msg.noun_num ; i++){
      aa = "";
      aa = ext_msg.noun[i];
	
      root = value();
      c = 0;
 
      ifstream stream("/home/yhirai/catkin_ws/src/noun/src/noun.json");
      stream >> root;
	
      const picojson::value::object& obj = root.get<picojson::object>();
      for (picojson::value::object::const_iterator i = obj.begin(); i != obj.end(); ++i) {
	json_map.insert( make_pair(i->first,i->second.to_str() ) );
      }
      c = json_map.count(aa);
      if(c > 0){
	object image = root.get<object>()[aa].get<object>();
	command_msg.data = image["command"].get<string>();
	name_com = image["word"].get<string>();
      }
    }
    if(command_msg.data != ""){
      command_pub.publish(command_msg);
      string output;
      cout << "Go to " << name_com << "'s position." << endl;
      output = name_com + "の場所に移動します";
      robot_browser(output,1);
    }
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

  void robot_browser(string action,int admin){
    zmq::context_t context(1);
    zmq::socket_t sender(context,ZMQ_PUSH);
    msgpack::sbuffer buffer;
    sender.connect("tcp://133.19.23.224:8081");
    map<string, string> dict;
    dict["senderName"] = "speechRecognition";
    dict["name"] = "Robot";
    dict["time"] = "";
    if(admin == 0)
      dict["speechRec"] = "あなたの命令は聞けません";
    else
      dict["speechRec"] = action;
    dict["place"] = "pioneer";
    msgpack::pack(buffer,dict);
    zmq::message_t message(buffer.size());
    memcpy(message.data(),buffer.data(),buffer.size());
    sender.send(message);
    
  }

  int robot_number(string noun){
    ifstream stream("/home/yhirai/catkin_ws/src/noun/src/robot.json");
    value root;
    int c;
    map<std::string,std::string> json_map;
    stream >> root;
    const picojson::value::object& obj = root.get<picojson::object>();
    for (picojson::value::object::const_iterator i = obj.begin(); i != obj.end(); ++i) {
      json_map.insert( make_pair(i->first,i->second.to_str() ) );
    }
    c = json_map.count(noun);

    if(c > 0){
      object image = root.get<object>()[noun].get<object>();
      double hh = image["robot_num"].get<double>();
      return hh;
    }
    else{
      return 0;
    }
  }

  void armdown(speech_msgs::Extract ext_msg,string name, string admin){
    int i;
    i = 0;
    ros::Subscriber ok_sub;
    cout << i << endl;
  }

  void baxterarm_up(speech_msgs::Extract ext_msg,string name,string admin){
    int robot = 0;
    if(admin != name){
      robot_browser("no",0);
      cout << "あなたじゃだめですよ" << endl;
      return;
    }
    string hh;
    std_msgs::String command_msg;
    ros::Publisher baxter_pub = nh.advertise<std_msgs::String>("baxter_chatter", 1000);
    for(int i = 0 ; i < ext_msg.noun_num ; i++){
      robot = robot_number(ext_msg.noun[i]);
      if(robot != 0)
	break;
    }
    map<std::string,std::string> json_map;
    value root;
    for(int i = 1; i < ext_msg.noun_num ; i++){
      string aa = "";
      hh = "";
      aa = ext_msg.noun[i];
      root = value();
      robot = robot_number(aa);
      int c = 0;
      ifstream stream("/home/yhirai/catkin_ws/src/noun/src/noun.json");
      stream >> root;

      const picojson::value::object& obj = root.get<picojson::object>();
      for (picojson::value::object::const_iterator i = obj.begin(); i != obj.end(); ++i) {
	json_map.insert( make_pair(i->first,i->second.to_str() ) );
      }
      c = json_map.count(aa);

      if(c > 0){
	object image = root.get<object>()[aa].get<object>();
	//    cout << "command=" << image["command"].get<string>() << endl;                                                                                 
	hh = image["command"].get<string>();
      }
      if(robot == 3 && hh == "righthand"){
	command_msg.data = "armup_r";
	cout << command_msg.data << endl;
	baxter_pub.publish(command_msg);
      }
      else if(robot == 3 && hh == "lefthand"){
	command_msg.data = "armup_l";
	cout << command_msg.data << endl;
	baxter_pub.publish(command_msg);
      }
      else if(robot == 3 && hh == "doublehand"){
	command_msg.data = "armup_d";
	cout << command_msg.data << endl;
	baxter_pub.publish(command_msg);
      }
    }
    
  }

  };

void check(){
  while(1){
    sleep(2);
    struct tm *pnow = localtime(&now);
    cout << pnow->tm_mon << endl;
    if(window_mood.size() != 0){
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
