#!/usr/bin/ruby

require 'gtk2'
require 'open3'
require 'WiringPiSpi'

Kakuchou_si   = '.pr6'
BasePrm  = "#{ENV['HOME']}/gtnprm"
PrjNames = "#{BasePrm}/prj_names#{Kakuchou_si}"
ComFonf  = "#{BasePrm}/com_config#{Kakuchou_si}"
SioFile  = "#{BasePrm}/sio#{Kakuchou_si}"
Prjs = [ "#{BasePrm}", 
        "#{BasePrm}/prj01", "#{BasePrm}/prj02", "#{BasePrm}/prj03", "#{BasePrm}/prj04", "#{BasePrm}/prj05",
        "#{BasePrm}/prj06", "#{BasePrm}/prj07", "#{BasePrm}/prj08", "#{BasePrm}/prj09", "#{BasePrm}/prj10",
        "#{BasePrm}/prj11", "#{BasePrm}/prj12", "#{BasePrm}/prj13", "#{BasePrm}/prj14", "#{BasePrm}/prj15",
        "#{BasePrm}/prj16", "#{BasePrm}/prj17", "#{BasePrm}/prj18", "#{BasePrm}/prj19", "#{BasePrm}/prj20" ]
File_ana = "#{ENV['HOME']}/Desktop/免疫ドライシステム#{Kakuchou_si}"
Led_conf = "#{ENV['HOME']}/Desktop/LED設定.txt"

$led_conf = []

$fact_a = 0.0256
$fact_b = -28.084

def hex2temp(a)
# return (a*0.0256-28.084).round(2)
  return (a*$fact_a+$fact_b).round(2)
end

def temp2hex(a)
# return ((a+28.084)/0.0256).to_i
  return ((a-$fact_b)/$fact_a).to_i
end

class Window
  attr_accessor :spi, :entAna, :lblLVal, :lblLA, :lblTVal, :lblCount, :lblTime, :lblTm2, :lblErr, :time_st, :time_st2, :entLED, :cbOnOff, :cbOverLed, :lblPumpVal

  def initialize
    @spi = WiringPiSpi.new
    @spi.setup_mode(1000000, 3)

    @time_st  = Process.clock_gettime(Process::CLOCK_MONOTONIC)
    @time_st2 = nil

    win = Gtk::Window.new()
    win.title= "LED電流調整"
    win.set_default_size(100,200)
    win.signal_connect(:destroy){exit_seq}

    lblAna   = Gtk::Label.new("号機");
    @entAna  = Gtk::Entry.new
    lbtn     = Gtk::Button.new.add(Gtk::Arrow.new(Gtk::Arrow::LEFT, Gtk::SHADOW_NONE))
    rbtn     = Gtk::Button.new.add(Gtk::Arrow.new(Gtk::Arrow::RIGHT, Gtk::SHADOW_NONE))
    lblLED   = Gtk::Label.new("LED電流値");
    @lblLVal = Gtk::Label.new("-");
    @lblLA   = Gtk::Label.new("-");
    @scrl    = Gtk::HScrollbar.new
    @scrl.set_range(0,0xfff)
    @scrl.set_value(2000)
    @entLED = Gtk::Entry.new
    @entLED.set_max_length(4)
    @entLED.set_xalign(1)
    @entLED.set_text("#{@scrl.value.floor}")
    @cbOnOff  = Gtk::CheckButton.new('LED ON')
    @cbOverLed= Gtk::CheckButton.new('過大光でLED OFF')
    @cbOverLed.active = true
    @lblCount = Gtk::Label.new("-");
    @lblTime  = Gtk::Label.new("-");
    btnSt     = Gtk::Button.new('START')
    btnStp    = Gtk::Button.new('STOP')
    @lblTm2   = Gtk::Label.new("-");
    @lblErr   = Gtk::Label.new("");

    lblGT      = Gtk::Label.new("ゲートタイム");
    @entGT     = Gtk::Entry.new
    @entGT.set_text("10")
    @entGT.set_xalign(1)
    lblGTUnit  = Gtk::Label.new("msec");
    lblLED_ON  = Gtk::Label.new("LED点灯時間");
    @entLED_ON = Gtk::Entry.new
    @entLED_ON.set_text("30")
    @entLED_ON.set_xalign(1)
    lblUnit1   = Gtk::Label.new("x0.1msec(0-99)");
    lblDelay   = Gtk::Label.new("ディレイ時間");
    @entDelay  = Gtk::Entry.new
    @entDelay.set_text("5")
    @entDelay.set_xalign(1)
    lblUnit2   = Gtk::Label.new("x0.1msec(0-99)");
    lblCntTime = Gtk::Label.new("カウント時間");
    @entCntTime = Gtk::Entry.new
    @entCntTime.set_text("10")
    @entCntTime.set_xalign(1)
    lblUnit3   = Gtk::Label.new("x0.1msec(0-99)");
    @lblLedErr = Gtk::Label.new("");
    lblMeasSt  = Gtk::Label.new("測光開始");
    @entMeasSt = Gtk::Entry.new
    @entMeasSt.set_text("10")
    @entMeasSt.set_xalign(1)
    lblMeasEd  = Gtk::Label.new("測光終了");
    @entMeasEd = Gtk::Entry.new
    @entMeasEd.set_text("10")
    @entMeasEd.set_xalign(1)

    lblTMP    = Gtk::Label.new("設定温度");
    @entTMP   = Gtk::Entry.new
    @entTMP.set_text("40")
    @entTMP.set_max_length(2)
    @entTMP.set_xalign(1)
    lblDo     = Gtk::Label.new("℃");
    @lblTVal  = Gtk::Label.new("-");
    lblTMPCfga  = Gtk::Label.new("温度係数a");
    @entTMPCfga = Gtk::Entry.new
    lblTMPCfgb  = Gtk::Label.new("温度係数b");
    @entTMPCfgb = Gtk::Entry.new
    @entTMPCfga.set_text("#{$fact_a}")
    @entTMPCfga.set_xalign(1)
    @entTMPCfgb.set_text("#{$fact_b}")
    @entTMPCfgb.set_xalign(1)

    lblPumpCfg  = Gtk::Label.new("ポンプ設定");
    @entPumpCfg = Gtk::Entry.new
    @entPumpCfg.set_text("7")
    @entPumpCfg.set_xalign(1)
    lblUnit4    = Gtk::Label.new("(1-15)初期値7");
    @lblPumpVal = Gtk::Label.new("-");

    # 分析機ファイルから呼び出して表示する
    if File.exist?( File_ana )
      open( File_ana, 'r' ) do |f|
        while line = f.gets
          ary = line.chop.split(/=/)
          case ary[0]
          when 'led_sv'
            @scrl.set_value(ary[1].to_i) if ary[1]
          when 'temp_sv'
            @entTMP.set_text("#{ary[1]}") if ary[1]
          when 'fact_a'
            @entTMPCfga.set_text("#{ary[1]}") if ary[1]
            $fact_a = @entTMPCfga.text.to_f
          when 'fact_b'
            @entTMPCfgb.set_text("#{ary[1]}") if ary[1]
            $fact_b = @entTMPCfgb.text.to_f
          when 'gate_time'
            @entGT.set_text("#{ary[1]}") if ary[1]
          when 'led_on_tm'
            @entLED_ON.set_text("#{ary[1]}") if ary[1]
          when 'led_delay'
            @entDelay.set_text("#{ary[1]}") if ary[1]
          when 'count_tm'
            @entCntTime.set_text("#{ary[1]}") if ary[1]
          when 'pump'
            @entPumpCfg.set_text("#{ary[1]}") if ary[1]
          when 'ana_no'
            @entAna.set_text("#{ary[1]}") if ary[1]
          when 'over_led'
            @cbOverLed.active = ("#{ary[1]}"=="true")?true:false if ary[1]
          when 'meas_st'
            @entMeasSt.set_text("#{ary[1]}") if ary[1]
          when 'meas_ed'
            @entMeasEd.set_text("#{ary[1]}") if ary[1]
          end
        end
      end
    end

    set_led_value
    set_led_config
    set_led_on_off(false)
    set_temp_value
    @spi.gate_time(@entGT.text.to_i)
    @spi.meas_points(@entMeasSt.text.to_i, @entMeasEd.text.to_i)

    # pump on/off
    value = @entPumpCfg.text.to_i
	value = 15 if value > 15
   #@spi.dataRW([0x2f,0x40,0x80|((value>>6)&0x7), 0xc0|(value&0x3f)])
    @spi.dataRW([0x2f,0x40,0x80|(value&0xf), 0xc0])

    @entTMP.signal_connect('changed') do 
      set_temp_value
    end

    @entTMPCfga.signal_connect('changed') do 
      $fact_a = @entTMPCfga.text.to_f
    end

    @entTMPCfgb.signal_connect('changed') do 
      $fact_b = @entTMPCfgb.text.to_f
    end

    @entLED.signal_connect('changed') do 
      i = @entLED.text.to_i
      @scrl.set_value(i)
    end

    @scrl.signal_connect('value-changed') do 
      set_led_value
    end

    lbtn.signal_connect('clicked') do 
      @scrl.set_value(@scrl.value-1)
    end

    rbtn.signal_connect('clicked') do 
      @scrl.set_value(@scrl.value+1)
    end

    @cbOnOff.signal_connect('clicked') do 
      set_led_on_off(@cbOnOff.active?)
    end

    @entLED_ON.signal_connect('changed') do 
      set_led_config
    end

    @entDelay.signal_connect('changed') do 
      set_led_config
    end

    @entCntTime.signal_connect('changed') do 
      set_led_config
    end

    btnSt.signal_connect('clicked') do 
      $led_conf = File.exist?(Led_conf) ? IO.readlines(Led_conf) : []
      @time_st2 = Process.clock_gettime(Process::CLOCK_MONOTONIC)
    end

    btnStp.signal_connect('clicked') do 
      $led_conf = []
      @time_st2 = nil
      @lblTm2.set_text("-");
    end

    @entGT.signal_connect('changed') do 
      i = @entGT.text.to_i
      @spi.gate_time(i)
    end

    @entMeasSt.signal_connect('changed') do 
      @spi.meas_points(@entMeasSt.text.to_i, @entMeasEd.text.to_i)
    end

    @entMeasEd.signal_connect('changed') do 
      @spi.meas_points(@entMeasSt.text.to_i, @entMeasEd.text.to_i)
    end

    @entPumpCfg.signal_connect('changed') do 
      # pump on/off
      value = @entPumpCfg.text.to_i
     #@spi.dataRW([0x2f,0x40,0x80|((value>>6)&0x7), 0xc0|(value&0x3f)])
	  value = 15 if value > 15
      @spi.dataRW([0x2f,0x40,0x80|(value&0xf), 0xc0])
    end

    tbl = Gtk::Table.new(2,3,true)
    tbl.set_column_spacings(3) 
    tbl.attach_defaults(@entAna,    0, 5, 0, 1)
    tbl.attach_defaults(lblAna,     5, 7, 0, 1)
    tbl.attach_defaults(lblLED,     0, 2, 1, 2)
    tbl.attach_defaults(@entLED,    2, 5, 1, 2)
    tbl.attach_defaults(@lblLVal,   6, 7, 1, 2)
    tbl.attach_defaults(@lblLA,     7, 8, 1, 2)
    tbl.attach_defaults(lbtn,       0, 1, 2, 3)
    tbl.attach_defaults(@scrl,      1, 7, 2, 3)
    tbl.attach_defaults(rbtn,       7, 8, 2, 3)
    tbl.attach_defaults(@cbOnOff,   2, 4, 3, 4)
    tbl.attach_defaults(@lblCount,  4, 6, 3, 4)
    tbl.attach_defaults(@lblTime,   6, 8, 3, 4)
    tbl.attach_defaults(@cbOverLed, 2, 8, 4, 5)
    tbl.attach_defaults(btnSt,      4, 6, 5, 6)
    tbl.attach_defaults(btnStp,     4, 6, 6, 7)
    tbl.attach_defaults(@lblTm2,    6, 8, 5, 6)
    tbl.attach_defaults(@lblErr,    6, 8, 6, 7)

    tbl.attach_defaults(lblGT,      0, 2, 8, 9)
    tbl.attach_defaults(@entGT,     2, 5, 8, 9)
    tbl.attach_defaults(lblGTUnit,  5, 6, 8, 9)
    tbl.attach_defaults(lblLED_ON,  0, 2, 9,10)
    tbl.attach_defaults(@entLED_ON, 2, 5, 9,10)
    tbl.attach_defaults(lblUnit1,   5, 7, 9,10)
    tbl.attach_defaults(lblDelay,   0, 2,10,11)
    tbl.attach_defaults(@entDelay,  2, 5,10,11)
    tbl.attach_defaults(lblUnit2,   5, 7,10,11)
    tbl.attach_defaults(lblCntTime, 0, 2,11,12)
    tbl.attach_defaults(@entCntTime,2, 5,11,12)
    tbl.attach_defaults(lblUnit3,   5, 7,11,12)
    tbl.attach_defaults(lblMeasSt,  0, 2,12,13)
    tbl.attach_defaults(@entMeasSt, 2, 5,12,13)
    tbl.attach_defaults(lblMeasEd,  0, 2,13,14)
    tbl.attach_defaults(@entMeasEd, 2, 5,13,14)
    tbl.attach_defaults(@lblLedErr, 2, 7,14,15)

    tbl.attach_defaults(lblTMP,     0, 2,15,16)
    tbl.attach_defaults(@entTMP,    2, 5,15,16)
    tbl.attach_defaults(lblDo,      5, 6,15,16)
    tbl.attach_defaults(@lblTVal,   6, 7,15,16)
    tbl.attach_defaults(lblTMPCfga, 0, 2,16,17)
    tbl.attach_defaults(@entTMPCfga,2, 5,16,17)
    tbl.attach_defaults(lblTMPCfgb, 0, 2,17,18)
    tbl.attach_defaults(@entTMPCfgb,2, 5,17,18)

    tbl.attach_defaults(lblPumpCfg, 0, 2,19,20)
    tbl.attach_defaults(@entPumpCfg,2, 5,19,20)
    tbl.attach_defaults(lblUnit4,   5, 7,19,20)
    tbl.attach_defaults(@lblPumpVal,6, 7,20,21)

    win.add(tbl)
    win.show_all()

  end

  def set_led_value
    value = @scrl.value.floor
    @entLED.set_text("#{value}") if @entLED.text.to_i != value
    # LED電流セット
    @spi.dataRW([0x28,0x40,0x80|((value>>6)&0x3f), 0xc0|(value&0x3f)])
  end

  # LED ON/OFF
  def set_led_on_off(active)
    @time_st = Process.clock_gettime(Process::CLOCK_MONOTONIC)

    # チェックボックスが外されたらレジスタ2をOFFにする
    rd = @spi.dataRW([0x2,0x40,0x80,0xc0])
    if active == false
      @spi.dataRW([0x22,rd[1],rd[2],rd[3]&~0x01])
    else
      # LED電流がゼロであればセットする
      ary = @spi.dataRW([0x08,0x40,0x80,0xc0])
      set_led_value if (ary[2]&~0x80)==0 && (ary[3]&~0xc0)==0

      @spi.dataRW([0x22,rd[1],rd[2],rd[3]|0x01])
    end
p [__LINE__, @spi.dataRW([0x2,0x40,0x80,0xc0])]

  end

  def set_led_config
    # LED点灯開始設定
    value = @entLED_ON.text.to_i + @entDelay.text.to_i + @entCntTime.text.to_i
    if value > 99
      @lblLedErr.set_text("設定時間の合計が99を超えました")
      style = Gtk::Style.new
      style.set_fg(Gtk::STATE_NORMAL, 65535, 0, 0)
      @lblLedErr.style = style
      value = 99
    else
      @lblLedErr.set_text("")
      style = Gtk::Style.new
      style.set_fg(Gtk::STATE_NORMAL, 0, 0, 0)
      @lblLedErr.style = style
    end
    @spi.dataRW([0x31,0x40,0x80|((value>>6)&0x3f), 0xc0|(value&0x3f)])
    # LED消灯開始設定
    value = @entDelay.text.to_i + @entCntTime.text.to_i
    @spi.dataRW([0x32,0x40,0x80|((value>>6)&0x3f), 0xc0|(value&0x3f)])
    # カウント時間設定
    value = @entCntTime.text.to_i
    @spi.dataRW([0x30,0x40,0x80|((value>>6)&0x3f), 0xc0|(value&0x3f)])
  end

  # 制御温度設定
  def set_temp_value
    value = @entTMP.text.to_i
    @entTMP.set_text("#{value}")
    value = temp2hex(value)
#p [__LINE__, "%02X %02X" % [0x80|((@scrl.value.floor>>6)&0x3f), 0xc0|(@scrl.value.floor&0x3f)]]
p [__LINE__, @spi.dataRW([0x29,0x40,0x80|((value>>6)&0x3f), 0xc0|(value&0x3f)])]
  end

  def exit_seq
    set_flag = [nil, nil, nil, nil, nil, nil, nil, nil, nil, nil, nil, nil, nil]
    file = []
    file = IO.readlines(File_ana) if File.exist?( File_ana )

    fw = open( File_ana, 'w' )
    file.each do |line|
      ary = line.chop.split(/=/)
      case ary[0]
      when 'led_sv'
        fw << "led_sv=#{@entLED.text.to_i}\n"
        set_flag[0] = true
      when 'temp_sv'
        fw << "temp_sv=#{@entTMP.text.to_i}\n"
        set_flag[1] = true
      when 'fact_a'
        fw << "fact_a=#{$fact_a}\n"
        set_flag[2] = true
      when 'fact_b'
        fw << "fact_b=#{$fact_b}\n"
        set_flag[3] = true
      when 'gate_time'
        fw << "gate_time=#{@entGT.text.to_i}\n"
        set_flag[4] = true
      when 'led_on_tm'
        fw << "led_on_tm=#{@entLED_ON.text.to_i}\n"
        set_flag[5] = true
      when 'led_delay'
        fw << "led_delay=#{@entDelay.text.to_i}\n"
        set_flag[6] = true
      when 'count_tm'
        fw << "count_tm=#{@entCntTime.text.to_i}\n"
        set_flag[7] = true
      when 'pump'
        fw << "pump=#{@entPumpCfg.text.to_i}\n"
        set_flag[8] = true
      when 'ana_no'
        fw << "ana_no=#{@entAna.text}\n"
        set_flag[9] = true
      when 'over_led'
        fw << "over_led=#{(@cbOverLed.active?)?"true":"false"}\n"
        set_flag[10] = true
      when 'meas_st'
        fw << "meas_st=#{@entMeasSt.text.to_i}\n"
        set_flag[11] = true
      when 'meas_ed'
        fw << "meas_ed=#{@entMeasEd.text.to_i}\n"
        set_flag[12] = true
      else
        fw << line
      end
    end

    fw << "led_sv=#{@entLED.text.to_i}\n" if set_flag[0] == nil
    fw << "temp_sv=#{@entTMP.text.to_i}\n" if set_flag[1] == nil
    fw << "fact_a=#{$fact_a}\n" if set_flag[2] == nil
    fw << "fact_b=#{$fact_b}\n" if set_flag[3] == nil
    fw << "gate_time=#{@entGT.text.to_i}\n" if set_flag[4] == nil
    fw << "led_on_tm=#{@entLED_ON.text.to_i}\n" if set_flag[5] == nil
    fw << "led_delay=#{@entDelay.text.to_i}\n" if set_flag[6] == nil
    fw << "count_tm=#{@entCntTime.text.to_i}\n" if set_flag[7] == nil
    fw << "pump=#{@entPumpCfg.text.to_i}\n" if set_flag[8] == nil
    fw << "ana_no=#{@entAna.text}\n" if set_flag[9] == nil
    fw << "over_led=#{(@cbOverLed.active?)?"true":"flase"}\n" if set_flag[10] == nil
    fw << "meas_st=#{@entMeasSt.text.to_i}\n" if set_flag[11] == nil
    fw << "meas_ed=#{@entMeasEd.text.to_i}\n" if set_flag[12] == nil
    fw.close

    Gtk.main_quit()
  end
end

w = Window.new

$count = [0, 1, 2, 3, 4]
$over  = 0
Gtk.timeout_add( 1000 ) do
  $count.map!{ |c| c+=1 }
#p [__LINE__, $count]

  # 過大光は赤色で
  ary = w.spi.dataRW([0x1f,0x40,0x80,0xc0])
  style = Gtk::Style.new
  style.set_fg(Gtk::STATE_NORMAL, ((ary[3]&0x02)==0x02)?65535:0, 0, 0)
  w.lblTime.style = style
  if ((ary[3]&0x02)==0x02)
    $over += 1
    # 過大光連続検知でSTOP
    if $over >= 5
      $led_conf = []
      w.time_st2 = nil
      style = Gtk::Style.new
      style.set_fg(Gtk::STATE_NORMAL, 65535, 0, 0)
      w.lblErr.style = style
      w.lblErr.set_text("過大光検知");
      w.cbOnOff.active = false if w.cbOverLed.active?
    end
  else
    $over = 0
  end
  # 通信エラー
  if (ary[0]==0 && ary[1]==0 && ary[2]==0 && ary[3]==0)
    style = Gtk::Style.new
    style.set_fg(Gtk::STATE_NORMAL, 65535, 0, 0)
    w.lblErr.style = style
    w.lblErr.set_text("通信エラー");
  elsif $over < 5
    w.lblErr.set_text("");
  end

  # gtn.rbプロセスがいなければSPIでフォトンカウントを取り込む
  o, e, s = Open3.capture3("ps -ax")
  unless /gtn.rb/ =~ o
    ary = w.spi.dataRW([0x14,0x40,0x80,0xc0])
    w.lblCount.set_text("#{((ary[1]&0x3f)<<12) + ((ary[2]&0x3f)<<6) + (ary[3]&0x3f)}")
    # errorの場合はリセットコマンドを発行する
    w.spi.dataRW([0x3f,0x40,0x80,0xc1]) if (ary[0]&0x20) == 0x20
  # gtnactionからフォトンカウント値を取得
  else
    w.lblCount.set_text("#{w.spi.foton_count}")
  end
  # 経過時刻表示
# tim = Time.at(Time.at(Time.now - w.time_st).getutc)
  tim = Time.at(Time.at(Process.clock_gettime(Process::CLOCK_MONOTONIC) - w.time_st).getutc)
  w.lblTime.set_text("#{tim.strftime('%H:%M:%S')}")
  if w.time_st2
#   tim = Time.at(Time.at(Time.now - w.time_st2).getutc)
    tim = Time.at(Time.at(Process.clock_gettime(Process::CLOCK_MONOTONIC) - w.time_st2).getutc)
    w.lblTm2.set_text("#{tim.strftime('%H:%M:%S')}")
  end

#p [__LINE__, w.lblTime.text, $led_conf[0]]
  # LED設定ファイルに従いLEDの設定値を変更する
  if ($led_conf[0] && $led_conf[0] != "\n" && w.lblTm2.text >= $led_conf[0].split(/\s/)[0])
    ary = $led_conf.shift.split(/\s+/)
    value = ary[2]
    w.entLED.set_text(value)
    w.cbOnOff.active = (ary[1] == 'OFF' || $over>=5) ? false:true if w.cbOverLed.active?
  end

  # 温度制御SV
  if false
    ary = w.spi.dataRW([0x09,0x40,0x80,0xc0])
    p [__LINE__, "#{((ary[2]<<6)&0xfc0)+(ary[3]&0x3f)}"] if ary[0]==1
  end

  # 温度
  if ($count[1]%5) == 0
    ary = w.spi.dataRW([0x06,0x40,0x80,0xc0])
    w.lblTVal.set_text("#{hex2temp(((ary[2]<<6)&0xfc0)+(ary[3]&0x3f))}") if ary[0]==1
  end

  # LED ON/OFF
  if ($count[2]%5) == 0
    ary = w.spi.dataRW([0x2,0x40,0x80,0xc0])
    w.cbOnOff.active = ((ary[3]&0x01)==0 || $over>=5) ? false:true if w.cbOverLed.active?
  end

  # LED制御SV
  if ($count[3]%5) == 0
    ary = w.spi.dataRW([0x08,0x40,0x80,0xc0])
    w.lblLVal.set_text("#{((ary[2]<<6)&0xfc0)+(ary[3]&0x3f)}") if ary[0]==1
  end

  # LED電流
  if ($count[4]%5) == 0
    ary = w.spi.dataRW([0x05,0x40,0x80,0xc0])
    w.lblLA.set_text("#{((ary[2]<<6)&0xfc0)+(ary[3]&0x3f)}") if ary[0]==1
  end

  # Pump
  if ($count[4]%5) == 0
    ary = w.spi.dataRW([0x0f,0x40,0x80,0xc0])
    w.lblPumpVal.set_text("#{ary[2]&0xf}") if ary[0]==1
  end
  true
end

Gtk.main()

