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

$shutter_run = nil
$shutter_value = [0, 0]


$fact_a = 0.0256
$fact_b = -28.084

$fact2_a = 0.0256
$fact2_b = -28.084

def hex2temp(a)
# return (a*0.0256-28.084).round(2)
  return (a*$fact_a+$fact_b).round(2)
end

def hex2temp2(a)
  return (a*$fact2_a+$fact2_b).round(2)
end

def temp2hex(a)
# return ((a+28.084)/0.0256).to_i
  return ((a-$fact_b)/$fact_a).to_i
end

class Window
  attr_accessor :spi, :entAna, :lblLVal, :lblLA, :lblTVal, :lblTVal2, :lblCount, :lblTime, :lblTm2, :lblErr, :time_st, :time_st2, :entLED, :cbOnOff, :cbOverLed, :lblPumpVal, :lblMeasDt, :lblStSts, :entDelay, :entCntTime, :entShtPos, :btnStop

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

    lblShut   = Gtk::Label.new("シャッタ：");
    @lblStSts = Gtk::Label.new("停止中");
    btnInit   = Gtk::Button.new('イニシャル')
    @rbtn1     = Gtk::RadioButton.new('全開')
    @rbtn2     = Gtk::RadioButton.new(@rbtn1, '全閉')
    @rbtn3     = Gtk::RadioButton.new(@rbtn1, '原点')
    hbox      = Gtk::HBox.new()
    hbox.pack_start(@rbtn1)
    hbox.pack_start(@rbtn2)
    hbox.pack_start(@rbtn3)
    @rbtn3.active = true
    @btnStop  = Gtk::Button.new('シャッタ動作')
    lblInit   = Gtk::Label.new("開閾値");
    @entInit  = Gtk::Entry.new
    @entInit.set_text("5")
    @entInit.set_xalign(1)
    lblShtPos = Gtk::Label.new("シャッタ");
    lblUnit5   = Gtk::Label.new("x0.1msec(0-99)");
    @entShtPos  = Gtk::Entry.new
    @entShtPos.set_text("5")
    @entShtPos.set_xalign(1)

    lblMeasSt1  = Gtk::Label.new("測光 1開始");
    @entMeasSt1 = Gtk::Entry.new
    @entMeasSt1.set_text("10")
    @entMeasSt1.set_xalign(1)
    lblMeasEd1  = Gtk::Label.new("~終了");
    @entMeasEd1 = Gtk::Entry.new
    @entMeasEd1.set_text("10")
    @entMeasEd1.set_xalign(1)
    lblMeasSt2  = Gtk::Label.new("測光 2開始");
    @entMeasSt2 = Gtk::Entry.new
    @entMeasSt2.set_text("10")
    @entMeasSt2.set_xalign(1)
    lblMeasEd2  = Gtk::Label.new("~終了");
    @entMeasEd2 = Gtk::Entry.new
    @entMeasEd2.set_text("10")
    @entMeasEd2.set_xalign(1)
    @lblMeasDt = Gtk::Label.new("");

    lblTMP    = Gtk::Label.new("設定温度");
    @entTMP   = Gtk::Entry.new
    @entTMP.set_text("40")
    @entTMP.set_max_length(2)
    @entTMP.set_xalign(1)
    lblDo     = Gtk::Label.new("℃");
    @lblTVal  = Gtk::Label.new("-");
    lblTMPCfg1a  = Gtk::Label.new("温度係数1a");
    @entTMPCfg1a = Gtk::Entry.new
    lblTMPCfg1b  = Gtk::Label.new("1b");
    @entTMPCfg1b = Gtk::Entry.new
    @entTMPCfg1a.set_text("#{$fact_a}")
    @entTMPCfg1a.set_xalign(1)
    @entTMPCfg1b.set_text("#{$fact_b}")
    @entTMPCfg1b.set_xalign(1)
    lblTMPCfg2a  = Gtk::Label.new("温度係数2a");
    @entTMPCfg2a = Gtk::Entry.new
    @lblTVal2 = Gtk::Label.new("-");
    lblTMPCfg2b  = Gtk::Label.new("2b");
    @entTMPCfg2b = Gtk::Entry.new
    @entTMPCfg2a.set_text("#{$fact2_a}")
    @entTMPCfg2a.set_xalign(1)
    @entTMPCfg2b.set_text("#{$fact2_b}")
    @entTMPCfg2b.set_xalign(1)

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
            @entTMPCfg1a.set_text("#{ary[1]}") if ary[1]
            $fact_a = @entTMPCfg1a.text.to_f
          when 'fact_b'
            @entTMPCfg1b.set_text("#{ary[1]}") if ary[1]
            $fact_b = @entTMPCfg1b.text.to_f
          when 'fact2_a'
            @entTMPCfg2a.set_text("#{ary[1]}") if ary[1]
            $fact2_a = @entTMPCfg2a.text.to_f
          when 'fact2_b'
            @entTMPCfg2b.set_text("#{ary[1]}") if ary[1]
            $fact2_b = @entTMPCfg2b.text.to_f
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
            @entMeasSt1.set_text("#{ary[1]}") if ary[1]
          when 'meas_ed'
            @entMeasEd1.set_text("#{ary[1]}") if ary[1]
          when 'meas_st2'
            @entMeasSt2.set_text("#{ary[1]}") if ary[1]
          when 'meas_ed2'
            @entMeasEd2.set_text("#{ary[1]}") if ary[1]
          when 'shutter_init'
            @entInit.set_text("#{ary[1]}") if ary[1]
          when 'shutter_pos'
            @entShtPos.set_text("#{ary[1]}") if ary[1]
          end
        end
      end
    end

    set_led_value
    set_led_config
    set_led_on_off(false)
    set_temp_value
    @spi.gate_time(@entGT.text.to_i)
    @spi.meas_points(@entMeasSt1.text.to_i, @entMeasEd1.text.to_i, @entMeasSt2.text.to_i, @entMeasEd2.text.to_i)
    @spi.fact2($fact_a, $fact_b, $fact2_a, $fact2_b)

    # pump on/off
    value = @entPumpCfg.text.to_i
    value = 15 if value > 15
   #@spi.dataRW([0x2f,0x40,0x80|((value>>6)&0x7), 0xc0|(value&0x3f)])
    @spi.dataRW([0x2f,0x40,0x80|(value&0xf), 0xc0])

    @entTMP.signal_connect('changed') do 
      set_temp_value
    end

    @entTMPCfg1a.signal_connect('changed') do 
      $fact_a = @entTMPCfg1a.text.to_f
      @spi.fact2($fact_a, $fact_b, $fact2_a, $fact2_b)
    end

    @entTMPCfg1b.signal_connect('changed') do 
      $fact_b = @entTMPCfg1b.text.to_f
      @spi.fact2($fact_a, $fact_b, $fact2_a, $fact2_b)
    end

    @entTMPCfg2a.signal_connect('changed') do 
      $fact2_a = @entTMPCfg2a.text.to_f
      @spi.fact2($fact_a, $fact_b, $fact2_a, $fact2_b)
    end

    @entTMPCfg2b.signal_connect('changed') do 
      $fact2_b = @entTMPCfg2b.text.to_f
      @spi.fact2($fact_a, $fact_b, $fact2_a, $fact2_b)
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

    @entMeasSt1.signal_connect('changed') do 
      @spi.meas_points(@entMeasSt1.text.to_i, @entMeasEd1.text.to_i, @entMeasSt2.text.to_i, @entMeasEd2.text.to_i)
    end

    @entMeasEd1.signal_connect('changed') do 
      @spi.meas_points(@entMeasSt1.text.to_i, @entMeasEd1.text.to_i, @entMeasSt2.text.to_i, @entMeasEd2.text.to_i)
    end

    @entMeasSt2.signal_connect('changed') do 
      @spi.meas_points(@entMeasSt1.text.to_i, @entMeasEd1.text.to_i, @entMeasSt2.text.to_i, @entMeasEd2.text.to_i)
    end

    @entMeasEd2.signal_connect('changed') do 
      @spi.meas_points(@entMeasSt1.text.to_i, @entMeasEd1.text.to_i, @entMeasSt2.text.to_i, @entMeasEd2.text.to_i)
    end

    # シャッタイニシャル
    btnInit.signal_connect('clicked') do 
      shutter_init
      ary = @spi.dataRW([0x16,0x40,0x80,0xc0])
      @btnStop.set_label('停止') if ary[0]==1 && (ary[1]&0x08)!=0x00
    end

    @entShtPos.signal_connect('changed') do 
      ary = @spi.dataRW([0x16,0x40,0x80,0xc0])
      if ary[0]==1 && (ary[1]&0x09)==0x01 && ($shutter_value[1]-$shutter_value[0])==25
        # シャッタ動作開始
        value = @entShtPos.text.to_i
        #value = @entDelay.text.to_i + @entCntTime.text.to_i
        @spi.dataRW([0x36,0x41,0x80|((value>>6)&0x3f),0xc0|(value&0x3f)])
        @lblStSts.set_text("Run 開2:#{$shutter_value[1]} 開1:#{$shutter_value[0]}")
        style = Gtk::Style.new
        style.set_fg(Gtk::STATE_NORMAL, 0, 0, 65535)
        @lblStSts.style = style

        @btnStop.set_label('停止')
      end
    end

    # シャッタ停止
    @btnStop.signal_connect('clicked') do 
      ary = @spi.dataRW([0x16,0x40,0x80,0xc0])
      if ary[0]==1 && (ary[1]&0x09)==0x00 && ($shutter_value[1]-$shutter_value[0])!=25
        # シャッタイニシャル
        shutter_init
        ary = @spi.dataRW([0x16,0x40,0x80,0xc0])
        @btnStop.set_label('停止') if ary[0]==1 && (ary[1]&0x08)!=0x00

      elsif ary[0]==1 && (ary[1]&0x09)==0x00 && ($shutter_value[1]-$shutter_value[0])==25
        # シャッタ動作開始
        value = @entShtPos.text.to_i
        #value = @entDelay.text.to_i + @entCntTime.text.to_i
        @spi.dataRW([0x36,0x41,0x80|((value>>6)&0x3f),0xc0|(value&0x3f)])
        @lblStSts.set_text("Run 開2:#{$shutter_value[1]} 開1:#{$shutter_value[0]}")
        style = Gtk::Style.new
        style.set_fg(Gtk::STATE_NORMAL, 0, 0, 65535)
        @lblStSts.style = style

        @btnStop.set_label('停止')

      elsif ary[0]==1 && (ary[1]&0x08)==0x00
        # 原点停止
        if @rbtn3.active?
          @spi.dataRW([0x36,0x42,0x80,0xc0])
          @lblStSts.set_text('停止中')
          style = Gtk::Style.new
          style.set_fg(Gtk::STATE_NORMAL, 0, 0, 0)
          @lblStSts.style = style
          @btnStop.set_label('シャッタ動作')
        # 全開停止
        elsif @rbtn1.active? && (($shutter_value[1]-$shutter_value[0])==25)
          value = ($shutter_value[0] + 6) * 4 / 5
          @spi.dataRW([0x36,0x42,0x80|((value>>6)&0x3f),0xc0|(value&0x3f)])
          @lblStSts.set_text('停止中')
          style = Gtk::Style.new
          style.set_fg(Gtk::STATE_NORMAL, 0, 0, 0)
          @lblStSts.style = style
          @btnStop.set_label('シャッタ動作')
        # 全閉停止
        elsif (($shutter_value[1]-$shutter_value[0])==25)
          value = ($shutter_value[1] > 1) ? ($shutter_value[1] - 2) * 4 / 5 : 0
          @spi.dataRW([0x36,0x42,0x80|((value>>6)&0x3f),0xc0|(value&0x3f)])
          @lblStSts.set_text('停止中')
          style = Gtk::Style.new
          style.set_fg(Gtk::STATE_NORMAL, 0, 0, 0)
          @lblStSts.style = style
          @btnStop.set_label('シャッタ動作')
        end
      end
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
    tbl.attach_defaults(lblAna,     5, 6, 0, 1)
    tbl.attach_defaults(lblLED,     0, 2, 1, 2)
    tbl.attach_defaults(@entLED,    2, 5, 1, 2)
    tbl.attach_defaults(@lblLVal,   5, 6, 1, 2)
    tbl.attach_defaults(@lblLA,     6, 7, 1, 2)
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

    tbl.attach_defaults(lblGT,      0, 2, 7, 8)
    tbl.attach_defaults(@entGT,     2, 5, 7, 8)
    tbl.attach_defaults(lblGTUnit,  5, 6, 7, 8)
    tbl.attach_defaults(lblLED_ON,  0, 2, 8, 9)
    tbl.attach_defaults(@entLED_ON, 2, 5, 8, 9)
    tbl.attach_defaults(lblUnit1,   5, 7, 8, 9)
    tbl.attach_defaults(lblDelay,   0, 2, 9,10)
    tbl.attach_defaults(@entDelay,  2, 5, 9,10)
    tbl.attach_defaults(lblUnit2,   5, 7, 9,10)
    tbl.attach_defaults(lblCntTime, 0, 2,10,11)
    tbl.attach_defaults(@entCntTime,2, 5,10,11)
    tbl.attach_defaults(lblUnit3,   5, 7,10,11)

    tbl.attach_defaults(lblShut,     0, 2,12,13)
    tbl.attach_defaults(@lblStSts,   2, 5,12,13)
    tbl.attach_defaults(btnInit,     0, 2,13,14)
    tbl.attach_defaults(hbox,        2, 6,13,14)
    tbl.attach_defaults(@btnStop,    6, 8,13,14)
    tbl.attach_defaults(lblInit,     0, 2,14,15)
    tbl.attach_defaults(@entInit,    2, 5,14,15)
    tbl.attach_defaults(lblShtPos,   0, 2,15,16)
    tbl.attach_defaults(@entShtPos,  2, 5,15,16)
    tbl.attach_defaults(lblUnit5,    5, 7,15,16)

    tbl.attach_defaults(lblMeasSt1,  0, 2,17,18)
    tbl.attach_defaults(@entMeasSt1, 2, 5,17,18)
    tbl.attach_defaults(lblMeasEd1,  5, 6,17,18)
    tbl.attach_defaults(@entMeasEd1, 6, 9,17,18)
    tbl.attach_defaults(lblMeasSt2,  0, 2,18,19)
    tbl.attach_defaults(@entMeasSt2, 2, 5,18,19)
    tbl.attach_defaults(lblMeasEd2,  5, 6,18,19)
    tbl.attach_defaults(@entMeasEd2, 6, 9,18,19)
    tbl.attach_defaults(@lblMeasDt,  2, 4,19,20)
    tbl.attach_defaults(@lblLedErr,  2, 7,20,21)

    tbl.attach_defaults(lblTMP,      0, 2,21,22)
    tbl.attach_defaults(@entTMP,     2, 5,21,22)
    tbl.attach_defaults(lblDo,       5, 6,21,22)
    tbl.attach_defaults(@lblTVal,    6, 7,21,22)
    tbl.attach_defaults(lblTMPCfg1a, 0, 2,22,23)
    tbl.attach_defaults(@entTMPCfg1a,2, 5,22,23)
    tbl.attach_defaults(lblTMPCfg1b, 5, 6,22,23)
    tbl.attach_defaults(@entTMPCfg1b,6, 9,22,23)
    tbl.attach_defaults(@lblTVal2,   9,10,22,23)
    tbl.attach_defaults(lblTMPCfg2a, 0, 2,23,24)
    tbl.attach_defaults(@entTMPCfg2a,2, 5,23,24)
    tbl.attach_defaults(lblTMPCfg2b, 5, 6,23,24)
    tbl.attach_defaults(@entTMPCfg2b,6, 9,23,24)

    tbl.attach_defaults(lblPumpCfg,  0, 2,25,26)
    tbl.attach_defaults(@entPumpCfg, 2, 5,25,26)
    tbl.attach_defaults(lblUnit4,    5, 7,25,26)
    tbl.attach_defaults(@lblPumpVal, 8, 9,25,26)

    win.add(tbl)
    win.show_all()

  end

  def shutter_init
    ary = @spi.dataRW([0x16,0x40,0x80,0xc0])
    if ary[0]==1 && (ary[1]&0x08)==0x00
      set_led_value
      value = @entInit.text.to_i
      @spi.dataRW([0x36,0x48,0x80|((value>>6)&0x3f),0xc0|(value&0x3f)])
      $shutter_run = 0
    end
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
    set_flag = [nil, nil, nil, nil, nil, nil, nil, nil, nil, nil, nil, nil, nil, nil, nil, nil, nil, nil, nil]
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
        fw << "meas_st=#{@entMeasSt1.text.to_i}\n"
        set_flag[11] = true
      when 'meas_ed'
        fw << "meas_ed=#{@entMeasEd1.text.to_i}\n"
        set_flag[12] = true
      when 'fact2_a'
        fw << "fact2_a=#{$fact2_a}\n"
        set_flag[13] = true
      when 'fact2_b'
        fw << "fact2_b=#{$fact2_b}\n"
        set_flag[14] = true
      when 'meas_st2'
        fw << "meas_st2=#{@entMeasSt2.text.to_i}\n"
        set_flag[15] = true
      when 'meas_ed2'
        fw << "meas_ed2=#{@entMeasEd2.text.to_i}\n"
        set_flag[16] = true
      when 'shutter_init'
        fw << "shutter_init=#{@entInit.text.to_i}\n"
        set_flag[17] = true
      when 'shutter_pos'
        fw << "shutter_pos=#{@entShtPos.text.to_i}\n"
        set_flag[18] = true
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
    fw << "meas_st=#{@entMeasSt1.text.to_i}\n" if set_flag[11] == nil
    fw << "meas_ed=#{@entMeasEd1.text.to_i}\n" if set_flag[12] == nil
    fw << "fact2_a=#{$fact2_a}\n" if set_flag[13] == nil
    fw << "fact2_b=#{$fact2_b}\n" if set_flag[14] == nil
    fw << "meas_st2=#{@entMeasSt2.text.to_i}\n" if set_flag[15] == nil
    fw << "meas_ed2=#{@entMeasEd2.text.to_i}\n" if set_flag[16] == nil
    fw << "shutter_init=#{@entInit.text.to_i}\n" if set_flag[17] == nil
    fw << "shutter_pos=#{@entShtPos.text.to_i}\n" if set_flag[18] == nil
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

  # 測定値データ
  style = Gtk::Style.new
  style.set_fg(Gtk::STATE_NORMAL, 0, 0, 65535)
  w.lblMeasDt.style = style
  w.lblMeasDt.set_text("測定値: #{w.spi.meas_data1}: #{w.spi.meas_data2}")

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

  # 温度(反応部)
  if ($count[1]%5) == 0
    ary = w.spi.dataRW([0x06,0x40,0x80,0xc0])
    w.lblTVal.set_text("#{hex2temp(((ary[2]<<6)&0xfc0)+(ary[3]&0x3f))}") if ary[0]==1
  end

  # 温度(装置内部)
  if ($count[1]%5) == 0
    ary = w.spi.dataRW([0x07,0x40,0x80,0xc0])
    w.lblTVal2.set_text("#{hex2temp2(((ary[2]<<6)&0xfc0)+(ary[3]&0x3f))}") if ary[0]==1
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

  # シャッタ
  if $shutter_run == 5
    ary = w.spi.dataRW([0x16,0x40,0x80,0xc0])
    if ary[0]==1 && (ary[1]&0x08)==0x00
      $shutter_value[0] = (ary[0]==1) ? ary[3]&0x3f : 0
      $shutter_value[1] = (ary[0]==1) ? ary[2]&0x3f : 0

      if ($shutter_value[1]-$shutter_value[0])==25
        # シャッタ動作開始
        value = w.entShtPos.text.to_i
        #value = w.entDelay.text.to_i + w.entCntTime.text.to_i
        w.spi.dataRW([0x36,0x41,0x80|((value>>6)&0x3f),0xc0|(value&0x3f)])
        w.lblStSts.set_text("Run 開2:#{$shutter_value[1]} 開1:#{$shutter_value[0]}")
        style = Gtk::Style.new
        style.set_fg(Gtk::STATE_NORMAL, 0, 0, 65535)
        w.lblStSts.style = style
        w.btnStop.set_label('停止')
      else
        w.lblStSts.set_text("Fail 開2:#{$shutter_value[1]} 開1:#{$shutter_value[0]}")
        style = Gtk::Style.new
        style.set_fg(Gtk::STATE_NORMAL, 65535, 0, 0)
        w.lblStSts.style = style
        w.btnStop.set_label('シャッタ動作')
      end
      $shutter_run = nil
    elsif ary[0]!=1
      w.lblStSts.set_text('Error')
      style = Gtk::Style.new
      style.set_fg(Gtk::STATE_NORMAL, 65535, 0, 0)
      w.lblStSts.style = style

      $shutter_run = nil
      $shutter_value[0] = 0
      $shutter_value[1] = 0
      w.btnStop.set_label('シャッタ動作')
    end
  elsif $shutter_run
    w.lblStSts.set_text('イニシャル中')
    style = Gtk::Style.new
    style.set_fg(Gtk::STATE_NORMAL, 0, 0, 0)
    w.lblStSts.style = style
    $shutter_run += 1

  end

  true
end

Gtk.main()

