#!/usr/bin/ruby

require 'gtk2'
#require 'kconv'
require 'fileutils'
require 'socket'
require 'gmail'

Gtk.init

VER = '1.00'

#MailTo = "konparus@alice.aandt.co.jp, yoshihara@alice.aandt.co.jp"
#MailTo = "konparus@alice.aandt.co.jp"
MailTo = nil

# 環境
#   ラズパイ3b+ ラズビアン + gtk2
# 拡張子履歴
# .pr4->.pr5 2005.07.16 ptableの先頭にuse追加
#            2007.06.15 actionの末尾にR6/R7/R8追加(S字対応)
# .pr5->.pr6 2016.05.04 モータ番号をﾓｰﾄﾞ番号＋モータ番号に変更
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

ACK = 0x06
NAK = 0x15
STX = 0x02
ETX = 0x03
ENQ = 0x05
EOT = 0x04
TimeOut = 3000

CommentTitle = 'コメント(先頭に#を付けると実行しません)'
IFCmt = 'if (RegA & Mask) == Value then goto'
UNCmt = 'if (RegA & Mask) != Value then goto'

CancelSt = '=無効開始'
CancelEd = '=無効終了'

ADDevFile = '/dev/adm686z'

$sio_dev = Array.new( 20, nil )
#$fd_ad   = nil
#$ad_log  = nil

# フォントを指定する
=begin
Gtk::RC::parse_string <<EndOfText
style "default"
{
  font_name="MS Gothic 16"
}
widget_class "*" style "default"
EndOfText
=end

$act_hash_ppm = {
  0x11 => 'Init.',
  0x12 => 'Step方向',
  0x13 => 'Home方向',
  0x14 => 'HOMEまで戻る',
  0x15 => '絶対パルス移動',
  0x16 => 'モーターの停止待ち',
  0x17 => '励磁解除',
  0x18 => '励磁ON',
# 0x31 => 'I/O Bit ON (0〜7:bit no)',
# 0x32 => 'I/O Bit OFF(0〜7:bit no)',
# 0x33 => 'I/O Bit Onを待つ',
# 0x34 => 'I/O Bit Offを待つ',
# 0x36 => 'I/O READ(確認用)=RegA'
}

$act_hash_dcm = {
  0x1 => 'Init.',
  0x2 => 'Step方向(msec)',
  0x3 => 'Step方向',
  0x4 => 'Home方向(msec)',
  0x5 => 'Home方向'
}

$act_hash_dio = {
# 0x41 => 'Bit ON (0〜31:bit no)',
  0x41 => 'Bit ON ',
# 0x42 => 'Bit OFF(0〜31:bit no)',
  0x42 => 'Bit OFF',
  0x43 => 'BitOnをまつ',
  0x44 => 'BitOffをまつ',
# 0x45 => '32bitポート書込み',
  0x46 => 'ポートREAD=RegA'
}

$act_hash_evt = {
  0x61 => 'Set Event',
  0x62 => 'Clr Event',
  0x63 => 'Wait Event Set',
  0x64 => 'Wait Event Clr'
}

$act_hash_adc = {
  0x71 => 'カウント取込み',
  0x72 => 'カウント取込終了待ち'
}

def b2d( str )
  h = 0
  ( 1..str.size ).each do |i|
    h = ( h << 1 ) + str[ i-1, 1 ].to_i
  end
  return h
end

class MySocket
  attr_reader :open_err

  def initialize
    @rcv_msg  = []
    @open_err = nil
    @port = TCPSocket.open("localhost",9001)
  end

  def calc_crc(m)
    crc = 0x84

    m[1..-3].each do |c|
      crc = ((((crc>>1)|(crc<<7)))&0xff)^c 
    end
    return crc
  end

  # NT1200(アーキテクト)フォーマットでデータ送信しACK/NACKを待つ
  def nt_send(sbuf, packstr)
    sbuf[-2] = calc_crc(sbuf)
#p [__LINE__, sbuf.pack(packstr), sbuf.pack(packstr).size ]
    @port.write( sbuf.pack(packstr) )
    @port.flush

    ret = nt_recv_ack_nack()
    @open_err = 1 if ret.size < 1
  end

  # ACK/NACK受信
  def nt_recv_ack_nack
    ret = []

    while 1 do
      ret = nt_recv()
      break if ret.size == 0
      break if ret[2] == 1 || ret[2] == 2 # ACK/NACK受信
      @rcv_msg.push ret
    end

    return ret
  end

  # メッセージ受信
  def nt_recv_each
    while select [@port], nil, nil, 0.001
      ret = nt_recv()
      break if ret.size == 0
      @rcv_msg.push ret if ret[2] != 1 && ret[2] != 2 # ACK/NACK以外を受信
    end
    
    @rcv_msg.each do |str|
      yield str
    end
    @rcv_msg = []
  end

  # NT1200(アーキテクト)フォーマットでデータ受信
  def nt_recv
    rbuf = []

    c = get_chr(TimeOut)        # STX
    rbuf.push c if c != nil

    if c == STX
      len = get_chr(TimeOut)    # length
      rbuf.push len if len != nil

      if len > ETX
        (2..len).each { rbuf.push get_chr(TimeOut) }
        c = get_chr(TimeOut)    # ETX
        rbuf.push c
      end
    end
    return rbuf
  end

  def get_chr(tm)
    (1..tm).each do |i|
      rs = select [@port], nil, nil, 0.001
      c = rs[0][0].read(1) if rs
      return c.unpack('C')[0] if c
    end
    return nil
  end
end

# DCモータレジスタの初期化
def dcmotor_reg_init

  return  if !File.exist?( $main_form.file_dcm )
  p "dcmotor_reg_init:#{$main_form.file_dcm}¥r"

  open( $main_form.file_dcm, "r" ) do |file|
    while line = file.gets
      ary = (line.chop).split( /,/ )
      next if ary[2] != '*'

      if $sock_port.open_err == nil
        cmd = [STX, 0x0e, 0x00, 0x00, 0xC103]
        cmd.push ary[0].to_i  # ノード番号
        cmd.push ary[1].to_i  # モーター番号
        cmd.push ary[3].to_i  # HomeActive
        cmd.push ary[4].to_i  # StepActive
        cmd.push ( (ary[5]=="CW") ? 0 : 1 ) # DirHomeOut
        cmd.push ( (ary[6]=="CW") ? 0 : 1 ) # DirHomeIn
        cmd.push ( (ary[7]=="CW") ? 0 : 1 ) # DirStep
        cmd.push ( (ary[8]=="CW") ? 0 : 1 ) # DirHome
        cmd.push 0            # CRC
        cmd.push ETX
        $sock_port.nt_send( cmd, 'C4nC10' )
      end
    end
  end
end

# パルスモータレジスタの初期化
def motor_reg_init

  return  if !File.exist?( $main_form.file_ppm )
  p "motor_reg_init:#{$main_form.file_ppm}¥r"

  open( $main_form.file_ppm, "r" ) do |file|
    while line = file.gets
      ary = (line.chop).split( /,/ )
      next if ary[2] != '*'

      if $sock_port.open_err == nil
        cmd = [STX, 0x21, 0x00, 0x00, 0xC101]
        cmd.push ary[0].to_i  # ノード番号
        cmd.push ary[1].to_i  # モーター番号
        cmd.push ary[3].to_i  # InitSpeed
        cmd.push ary[4].to_i  # Ratio
        cmd.push ary[5].to_i  # HomeOutPulse
        cmd.push ary[6].to_i  # HomeInPulse
        cmd.push ary[7].to_i  # HomeActive
        cmd.push ( (ary[8]=="CW") ? 0 : 1 ) # DirHomeOut
        cmd.push ( (ary[9]=="CW") ? 0 : 1 ) # DirHomeIn
        cmd.push ( (ary[10]=="CW") ? 0 : 1 )# DirStep
        cmd.push ( (ary[11]=="CW") ? 0 : 1 )# DirHome
        cmd.push ary[12].hex  # iR9
        cmd.push ary[13].hex  # iR10
        cmd.push ary[14].hex  # iR11
        cmd.push 0            # CRC
        cmd.push ETX
        $sock_port.nt_send( cmd, 'C4nC2n4C5N3C2' )
      end
    end
  end
end

# ACTION情報の送信
def action_info_send(console_no, ary)
  cmd = [STX, 0x20, 0x00, console_no, 0xC102]
  cmd.push ary[0].to_i  # 行番号
  cmd.push ary[3].to_i  # ノード番号
  cmd.push ary[4].to_i  # モータ番号
  cmd.push 0
  cmd.push ary[6].to_i  # 移動量
  cmd.push ary[7].to_i  # 自起動
  cmd.push ary[8].to_i  # 最高速
  cmd.push ary[9].to_i  # 加速
  cmd.push ary[10].to_i # 減速
  cmd.push ary[11].to_i # 倍率
  cmd.push ary[12].to_i # 減速開始
  cmd.push ary[13].to_i # S次加速
  cmd.push ary[14].to_i # S次減速
  cmd.push 0            # CRC
  cmd.push ETX

  case ary[2]
  when "PPM"
    cmd[8] = $act_hash_ppm.key(ary[5]) if $act_hash_ppm.key(ary[5])
  when "DCM"
    cmd[8] = $act_hash_dcm.key(ary[5]) if $act_hash_dcm.key(ary[5])
  when "DIO"
    cmd[8] = $act_hash_dio.key(ary[5]) if $act_hash_dio.key(ary[5])
    cmd[9] = ary[6].hex                if cmd[8] == 0x45 # 32bitポート書込み の場合は移動量は16進数で
  when "WAIT"
    cmd[8] = 0x51
  when "IF"
    cmd[ 8] = 0x52
    cmd[10] = ary[7].hex  # 自起動16進数(Mask)
    cmd[11] = ary[8].hex  # 最高速16進数(Value)
  when "UNLESS"
    cmd[ 8] = 0x53
    cmd[10] = ary[7].hex  # 自起動16進数(Mask)
    cmd[11] = ary[8].hex  # 最高速16進数(Value)
  when "GOTO"
    cmd[8] = (ary[5] == 'GOTO_END') ? 0x54 : 0x55
  when "SIO"
    cmd[8] = 0x56
  when "ERROR"
    cmd[8] = 0x57
  when "EVENT"
    cmd[8] = $act_hash_evt.key(ary[5]) if $act_hash_evt.key(ary[5])
  when "MEAS"
    cmd[ 8] = $act_hash_adc.key(ary[5]) if $act_hash_adc.key(ary[5])
    cmd[10] = ary[7].to_i      # 自起動(min 32bit)
    cmd[11] = ary[7].to_i>>16  # 自起動(min 32bit)
    cmd[12] = ary[8].to_i      # 最高速(max 32bit)
    cmd[13] = ary[8].to_i>>16  # 最高速(max 32bit)
  end
  $sock_port.nt_send( cmd, 'C4n2C2nNn8C2' )
end

# GPIO情報の取得
def gpio_info
  ret = 0

  # ファイルが無ければすべてINポート
  return ret unless File.exist?( $main_form.file_gpio )

  open( $main_form.file_gpio, "r" ) do |f|
    while line = f.gets
      ret = (ret << 1 ) + ((line == "IN\n") ? 0:1)
    end
  end
  return ret
end

# Main画面
class Gtn

  attr_accessor :prj_no, :console_opened, :enSerial, :status, :main_sts, :main_info, :file_tnet, :file_ppm, :file_dcm, :file_config, :file_gpio, :file_action, :file_ana

  def initialize()

    # 以下、画面には表示しないがソースコード共通化のために宣言する
    @size   = 20
    @console_opened = Array.new( @size+1, nil )
    @start  = []
    @stop   = []
    @loop   = []
    @run    = []
    @status = []
    ( 1..@size ).each do |i|
      tmp            = Gtk::Adjustment.new( 1, 1, 9999, 1, 2, 0 )
      @start[ i-1 ]  = Gtk::SpinButton.new( tmp, 0, 0 )
      tmp            = Gtk::Adjustment.new( 1, 1, 9999, 1, 2, 0 )
      @stop[ i-1 ]   = Gtk::SpinButton.new( tmp, 0, 0 )
      tmp            = Gtk::Adjustment.new( 1, 0, 999999, 1, 2, 0 )
      @loop[ i-1 ]   = Gtk::SpinButton.new( tmp, 0, 0 )
      @run[ i-1 ]    = Gtk::CheckButton.new( "run" )
      @status[ i-1 ] = Gtk::Label.new( "stop" )
    end

    # main Windowの作成
    form = Gtk::Window.new()
    form.set_title( '免疫ドライシステム Ver.' + VER );
    form.signal_connect( 'delete_event' ){ exit_seq }
    form.window_position = Gtk::Window::POS_CENTER
    # メニューの作成
    @enSerial  = Gtk::Entry.new()
    gouki      = Gtk::Label.new('号機番号:')
    btnStop    = Gtk::Button.new( '停止' )
    btnCntRcv  = Gtk::Button.new( 'カウント取込み' )
    btnStart   = Gtk::Button.new( '測定開始' )
    btnPrimeA  = Gtk::Button.new( 'プライム' )
    btnPrimeB  = Gtk::Button.new( 'プライム(カード使用)' )
    btnInit    = Gtk::Button.new( 'イニシャライズ' )
    btnNZUp    = Gtk::Button.new( 'ノズルUP' )
    btnNZDn    = Gtk::Button.new( 'ノズルDown' )
    btnSyrUp   = Gtk::Button.new( 'シリンジUP' )
    btnSyrDn   = Gtk::Button.new( 'シリンジDown' )
    btnMixMove = Gtk::Button.new( '撹拌回転' )
    @main_sts  = Gtk::Label.new( "" )
    @main_info = Gtk::Label.new( "" )

    table = Gtk::Table.new( 1, 6, true )
    table.attach( gouki,        0, 1, 0, 1 )
    table.attach( @enSerial,    1, 2, 0, 1 )
    table.attach( btnCntRcv,    0, 1, 1, 5 )
    table.attach( btnPrimeA,    1, 2, 1, 5 )
    table.attach( btnStart,     0, 1, 5, 9 )
    table.attach( btnPrimeB,    1, 2, 5, 9 )

    table.attach( btnInit,      3, 4, 1, 4 )
    table.attach( btnMixMove,   4, 5, 1, 4 )
    table.attach( btnNZUp,      3, 4, 4, 7 )
    table.attach( btnSyrUp,     4, 5, 4, 7 )
    table.attach( btnNZDn,      3, 4, 7,10 )
    table.attach( btnSyrDn,     4, 5, 7,10 )

    table.attach( btnStop,      5, 6, 1,10 )


    table.attach( @main_sts,    1, 5,10,11 )
    table.attach( @main_info,   1, 5,11,12 )

    form.add table
    form.show_all

    # シグナル定義
    btnInit.signal_connect( 'clicked' ){ init_clicked }
    btnNZUp.signal_connect( 'clicked' ){ nozzle_up_clicked }
    btnNZDn.signal_connect( 'clicked' ){ nozzle_dn_clicked }
    btnSyrUp.signal_connect( 'clicked' ){ syringe_up_clicked }
    btnSyrDn.signal_connect( 'clicked' ){ syringe_dn_clicked }
    btnMixMove.signal_connect( 'clicked' ){ mix_Move_clicked }
    btnPrimeA.signal_connect( 'clicked' ){ prime_a_clicked }
    btnPrimeB.signal_connect( 'clicked' ){ prime_b_clicked }
    btnCntRcv.signal_connect( 'clicked' ){ count_clicked }
    btnStart.signal_connect( 'clicked' ){ start_clicked }
    btnStop.signal_connect( 'clicked' ){ stop_clicked }
  end

  # Init
  def init_clicked
    # 1番
    @run[0].active = true
    execute
    @run[0].active = false
  end

  # ノズルUP
  def nozzle_up_clicked
    # 9番
    @run[8].active = true
    execute
    @run[8].active = false
  end

  # ノズルDown
  def nozzle_dn_clicked
    # 10番
    @run[9].active = true
    execute
    @run[9].active = false
  end

  # シリンジUP
  def syringe_up_clicked
    # 11番
    @run[10].active = true
    execute
    @run[10].active = false
  end

  # シリンジDown
  def syringe_dn_clicked
    # 12番
    @run[11].active = true
    execute
    @run[11].active = false
  end

  # 撹拌回転
  def mix_Move_clicked
    # 13番
    @run[12].active = true
    execute
    @run[12].active = false
  end

  # プライム
  def prime_a_clicked
    # 18番
    @run[17].active = true
    execute
    @run[17].active = false
  end

  # プライム(カード使用)
  def prime_b_clicked
    # 19番
    @run[18].active = true
    execute
    @run[18].active = false
  end

  # カウント受信
  def count_clicked
    # 8番
    @run[7].active = true
    execute
    @run[7].active = false
  end

  # Start
  def start_clicked
    # 2-5番
    @run[1].active = true
    @run[2].active = true
    @run[3].active = true
    @run[4].active = true
    execute
    @run[1].active = false
    @run[2].active = false
    @run[3].active = false
    @run[4].active = false
  end

  def execute
    # プロジェクトが選ばれている事
    return if $main_form.file_config == nil

    # 全Actionが停止であることを確認する
    return unless $main_form.status.map { |x| x.text }.uniq == ['stop']

    ready = []
    ( 1..@size ).each do |i|

      next if !@run[ i-1 ].active?

      if $sock_port.open_err == nil
        # ActionプロセスへREADY要求
        $sock_port.nt_send( [STX, 0x08, 0x00, 0x00, 0xC014, i, gpio_info(), 0x00, ETX], 'C4nC4' )
        ready.push i

        # Actionファイルを読み込む
        fname = $main_form.file_action + "#{i}" + Kakuchou_si
        if File.exist?( fname )
          open( fname, "r" ) do |f|
            cancel = nil
            while line = f.gets
              ary = (line.chop).split( /,/ )

              cancel = true if ary[5] == CancelSt
              cancel = nil  if ary[5] == CancelEd

              next if ary[0].to_i < @start[ i-1 ].value_as_int
              next if ary[0].to_i > @stop[ i-1 ].value_as_int
              next if ary[5] == CancelEd
              next if (ary[1])[ 0, 1 ] == "#"
              next if cancel

              action_info_send(i, ary)
            end
          end
        end
        style = Gtk::Style.new
        style.font_desc = Pango::FontDescription.new("Monospace 18")
       #style.set_fg(Gtk::STATE_NORMAL, 0, 65535, 0)
        style.set_fg(Gtk::STATE_NORMAL, 0, 60000, 0)
        $main_form.main_sts.style = style
        $main_form.main_sts.set_text( "RUN" )
      else
        $main_form.main_sts.set_text( "Socket送受信エラー!!" )
      end
    end

    # ActionプロセスへSTART要求
    ready.each do |i|
      $sock_port.nt_send( [STX, 0x11, 0x00, 0x00, 0xC015, i, @loop[ i-1 ].value_as_int,
                                                              0, 0, 0, 0, ETX], 'C4nCNn3C2' )
      $main_form.status[ i-1 ].set_text "run"
    end
  end

  # EMG
  def stop_clicked
    # プロジェクトが選ばれている事
    return if $main_form.file_config == nil

    $sock_port.nt_send( [STX, 0x07, 0x00, 0x00, 0xC016, 0, 0x00, ETX], 'C4nC3' ) if $sock_port.open_err == nil
  end

  def setup
    # プロジェクト環境変数
    $main_form.prj_no = 1
    $main_form.file_tnet   = "#{Prjs[$main_form.prj_no]}/tnet#{Kakuchou_si}"
    $main_form.file_ppm    = "#{Prjs[$main_form.prj_no]}/ptable#{Kakuchou_si}"
    $main_form.file_dcm    = "#{Prjs[$main_form.prj_no]}/dctable#{Kakuchou_si}"
    $main_form.file_gpio   = "#{Prjs[$main_form.prj_no]}/gpio#{Kakuchou_si}"
    $main_form.file_config = "#{Prjs[$main_form.prj_no]}/config#{Kakuchou_si}"
    $main_form.file_action = "#{Prjs[$main_form.prj_no]}/action"
    $main_form.file_ana    = "#{Prjs[$main_form.prj_no]}/免疫ドライシステム#{Kakuchou_si}"

    # スレッド通知
    $sock_port.nt_send( [STX, 0x08, 0x00, 0x00, 0xC012, $main_form.prj_no, gpio_info(), 0x00, ETX], 'C4nC4' ) if $sock_port.open_err == nil

    # モータレジスタの初期化
    motor_reg_init
    dcmotor_reg_init

    # configファイルから呼び出して表示する
    if File.exist?( $main_form.file_config )
      open( $main_form.file_config, 'r' ) do |f|
        while line = f.gets
          ary = line.chop.split( /,/ )
          i   = ary[0].to_i
          if i <= @size && ary.size == 8
           #@start[ i-1 ].set_value( ary[2].to_i )
           #@stop[ i-1 ].set_value( ary[3].to_i )
            @loop[ i-1 ].set_value( ary[4].to_i )
          end
        end
      end
    end

    # 分析機ファイルから呼び出して表示する
    if File.exist?( $main_form.file_ana )
      open( $main_form.file_ana, 'r' ) do |f|
        while line = f.gets
          ary = line.chop.split(/=/)
          case ary[0]
          when 'ana_no'
            @enSerial.set_text( ary[1] )
          end
        end
      end
    end

    # START/STOP行はactionファイルの開始行と終了行を表示する
    (1..@size).each do |i|
      fname = $main_form.file_action + "#{i}" + Kakuchou_si
      if File.exist?( fname )
        ary = IO.readlines(fname)
        @start[ i-1 ].set_value( ary[0].chop.split( /,/ )[0].to_i ) if ary[0]
        @stop[ i-1 ].set_value( ary[-1].chop.split( /,/ )[0].to_i ) if ary[-1]
      else
        @start[ i-1 ].set_value( 1 )
        @stop[ i-1 ].set_value( 1 )
      end
    end

    # 起動時Init.
    init_clicked
  end

  def exit_seq
    open($main_form.file_ana, 'w') do |f|
      f << "ana_no=#{@enSerial.text}\n"
    end
    Gtk::main_quit
  end
end


#-------------------------------------------------------------------------------
# ここからSTART
#-------------------------------------------------------------------------------

# 通信オブジェクト生成
$sock_port = MySocket.new

# MAIN画面表示
$main_form = Gtn.new
$main_form.setup

# RTタスクからの受信処理
Gtk.timeout_add( 200 ) do
  $sock_port.nt_recv_each do |rcv_msg|
    stx, len, type, dmy, id, my_no, dsp, line, msg, crc, etx = rcv_msg.pack('C*').unpack('C4nC3A32C2')

    # ベース画面のステータス表示
    if my_no == 0 && msg
      $main_form.main_info.set_text( msg )
    end

    # ベース画面の各状態をstopに
    if my_no > 0 && line == 1 && ( msg == 'success!!' || msg[0,3] == 'ERR' || msg[0,4] == 'STOP' )
      $main_form.status[ my_no-1 ].set_text( 'stop' )
      # 全てstopであれば全体ステータスを表示
      if $main_form.status.map { |x| x.text }.uniq == ['stop']
        style = Gtk::Style.new
        style.font_desc = Pango::FontDescription.new("Monospace 14")
        style.set_fg(Gtk::STATE_NORMAL, 0, 0, 0)
        $main_form.main_sts.style = style
        $main_form.main_sts.set_text( msg )
      end
    end

    # Action画面のステータス
    if my_no > 0 && dsp == 1 && msg && $main_form.console_opened[ my_no ]
      # メッセージ表示
      $main_form.console_opened[ my_no ].lblStatus[ line-1 ].set_text( msg )
      # エラーは赤文字
      if line == 1 && msg =~ /ERR/
        style = Gtk::Style.new
        style.font_desc = Pango::FontDescription.new("Monospace 14")
        style.set_fg(Gtk::STATE_NORMAL, 65535, 0, 0)
        $main_form.console_opened[ my_no ].lblStatus[ line-1 ].style = style

        # 行番号を取り出す
        msg =~ /ERR (.*) = (\d+)/
        eline = $2
        # Actionファイルを読み込んでコメントを取り出しステータス欄に表示する
        fname = $main_form.file_action + "#{my_no}" + Kakuchou_si
        if File.exist?( fname )
          open( fname, "r" ) do |f|
            while rline = f.gets
              ary = (rline.chop).split( /,/ )

              if ary[0].to_i == eline.to_i
                $main_form.console_opened[ my_no ].lblStatus[ line ].set_text( ary[1] )
                break
              end
            end
          end
        end
      # その他は黒文字
      else
        style = Gtk::Style.new
        style.font_desc = Pango::FontDescription.new("Monospace 14")
        style.set_fg(Gtk::STATE_NORMAL, 0, 0, 0)
        $main_form.console_opened[ my_no ].lblStatus[ line-1 ].style = style
      end

    # 次の行番号
    elsif my_no > 0 && dsp == 2 && msg && $main_form.console_opened[ my_no ]
      eline = msg.to_i

      list = []
      iter = $main_form.console_opened[ my_no ].treeview.model.iter_first
      begin
        list.push iter.get_value(0).to_i
      end while iter.next!
      
      # 最終行への処理
      eline = list[-1] if eline < 0

      # TreeViewから指定行を探す
      find = nil
      iter = $main_form.console_opened[ my_no ].treeview.model.iter_first
      begin
        # 見つけたらカーソル移動
        if (iter.get_value(0).to_i >= eline || iter.get_value(0).to_i == list[-1]) && find == nil
          $main_form.console_opened[ my_no ].treeview.selection.select_iter(iter)
          find = true
        # カーソル未選択
        else
          $main_form.console_opened[ my_no ].treeview.selection.unselect_iter(iter)
        end
      end while iter.next!
    end

    # A/D取り込みファイルのgmail送信
    if line == 1 && msg && MailTo
      if msg =~ /FILE/
        file = msg.split(/\s/)[-1]
        name = msg.split(/\//)[-1]
        # gmail送信
        gmail = Gmail.connect("aandtrandd@gmail.com","yvtogiqruxmurtxg")
        gmail.deliver do
          to MailTo
          subject "#{$main_form.enSerial.text}号機 測定レポート:#{name}"
          #text_part do
          #  body "本文"
          #end
          add_file file
        end
        gmail.logout
        File.unlink file
      end
    end
  end
  true
end

Gtk.main

