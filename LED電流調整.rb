#!/usr/bin/ruby

require 'gtk2'
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
File_ana = "#{Prjs[1]}/免疫ドライシステム#{Kakuchou_si}"

def hex2temp(a)
  return a*0.0256-28.084
end

def temp2hex(a)
  return ((a+28.084)/0.0256).floor(0)
end

class Window
  attr_accessor :spi, :lblLVal, :lblLA, :lblTVal

  def initialize
    @spi = WiringPiSpi.new
    @spi.setup_mode(1000000, 3)

    win = Gtk::Window.new()
    win.title= "LED電流調整"
    win.set_default_size(100,200)
    win.signal_connect(:destroy){exit_seq}

    lbtn     = Gtk::Button.new.add(Gtk::Arrow.new(Gtk::Arrow::LEFT, Gtk::SHADOW_NONE))
    rbtn     = Gtk::Button.new.add(Gtk::Arrow.new(Gtk::Arrow::RIGHT, Gtk::SHADOW_NONE))
    lblLED   = Gtk::Label.new("LED電流値");
    @lblLVal = Gtk::Label.new("-");
    @lblLA   = Gtk::Label.new("-");
    lblTMP   = Gtk::Label.new("設定温度");
    lblDo    = Gtk::Label.new("℃");
    @lblTVal = Gtk::Label.new("-");
    @scrl    = Gtk::HScrollbar.new
    @scrl.set_range(1,0xfff)
    @scrl.set_value(2000)
    @entLED = Gtk::Entry.new
    @entLED.set_max_length(4)
    @entLED.set_xalign(1)
    @entLED.set_text("#{@scrl.value.floor}")
    @entTMP = Gtk::Entry.new
    @entTMP.set_text("40")
    @entTMP.set_max_length(2)
    @entTMP.set_xalign(1)

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
          end
        end
      end
    end

    set_led_value
    set_temp_value

    @entTMP.signal_connect('changed') do 
      set_temp_value
    end

    @entLED.signal_connect('changed') do 
      i = @entLED.text.to_i
      @scrl.set_value(i) if i>0
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

    tbl = Gtk::Table.new(2,3,true)
    tbl.set_column_spacings(3) 
    tbl.attach_defaults(lblLED,   0, 2, 0, 1)
    tbl.attach_defaults(@entLED,  2, 5, 0, 1)
    tbl.attach_defaults(@lblLVal, 6, 7, 0, 1)
    tbl.attach_defaults(@lblLA,   7, 8, 0, 1)
    tbl.attach_defaults(lbtn,     0, 1, 1, 2)
    tbl.attach_defaults(@scrl,    1, 7, 1, 2)
    tbl.attach_defaults(rbtn,     7, 8, 1, 2)
    tbl.attach_defaults(lblTMP,   0, 2, 3, 4)
    tbl.attach_defaults(@entTMP,  2, 5, 3, 4)
    tbl.attach_defaults(lblDo,    5, 6, 3, 4)
    tbl.attach_defaults(@lblTVal, 6, 7, 3, 4)
    win.add(tbl)
    win.show_all()

  end

  def set_led_value
    value = @scrl.value.floor
    @entLED.set_text("#{value}")
#p [__LINE__, "%02X %02X" % [0x80|((@scrl.value.floor>>6)&0x3f), 0xc0|(@scrl.value.floor&0x3f)]]
p [__LINE__, @spi.dataRW([0x28,0x40,0x80|((value>>6)&0x3f), 0xc0|(value&0x3f)])]
  end

  def set_temp_value
    value = @entTMP.text.to_i
    @entTMP.set_text("#{value}")
#p [__LINE__, "%02X %02X" % [0x80|((@scrl.value.floor>>6)&0x3f), 0xc0|(@scrl.value.floor&0x3f)]]
p [__LINE__, @spi.dataRW([0x29,0x40,0x80|((value>>6)&0x3f), 0xc0|(value&0x3f)])]
  end

  def exit_seq
    set1_flag = nil
    set2_flag = nil
    file = []
    file = IO.readlines(File_ana) if File.exist?( File_ana )

    fw = open( File_ana, 'w' )
    file.each do |line|
      ary = line.chop.split(/=/)
      case ary[0]
      when 'led_sv'
        fw << "led_sv=#{@entLED.text.to_i}\n"
        set1_flag = true
      when 'temp_sv'
        fw << "temp_sv=#{@entTMP.text.to_i}\n"
        set2_flag = true
      else
        fw << line
      end
    end

    fw << "led_sv=#{@entLED.text.to_i}\n" if set1_flag == nil
    fw << "temp_sv=#{@entTMP.text.to_i}\n" if set2_flag == nil
    fw.close

    Gtk.main_quit()
  end
end

w = Window.new

Gtk.timeout_add( 1000 ) do
  # 温度制御SV
  ary = w.spi.dataRW([0x09,0x40,0x80,0xc0])
# p [__LINE__, "#{((ary[2]<<6)&0xfc0)+(ary[3]&0x3f)}"] if ary[0]==1

#=begin
  # 温度
  ary = w.spi.dataRW([0x06,0x40,0x80,0xc0])
  w.lblTVal.set_text("#{hex2temp(((ary[2]<<6)&0xfc0)+(ary[3]&0x3f))}") if ary[0]==1
#p [__LINE__, "#{hex2temp(((ary[2]<<6)&0xfc0)+(ary[3]&0x3f))}"] if ary[0]==1
  # LED制御SV
  ary = w.spi.dataRW([0x08,0x40,0x80,0xc0])
  w.lblLVal.set_text("#{((ary[2]<<6)&0xfc0)+(ary[3]&0x3f)}") if ary[0]==1
  # LED電流
  ary = w.spi.dataRW([0x05,0x40,0x80,0xc0])
  w.lblLA.set_text("#{((ary[2]<<6)&0xfc0)+(ary[3]&0x3f)}") if ary[0]==1
  # errorの場合はリセットコマンドを発行する
  w.spi.dataRW([0x3f,0x40,0x80,0xc1]) if ary[0]==0x20
#=end
end

Gtk.main()

