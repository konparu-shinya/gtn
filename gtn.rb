#!/usr/bin/ruby

require 'gtk2'
#require 'kconv'
#require 'FileUtils'
require 'socket'

Gtk.init

VER = '12.0-gtk2'

# 環境
#   ラズパイ3b+ CANコントローラ MCP2515 + ラズビアン + gtk2
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
  0x41 => 'Bit ON (0〜31:bit no)',
  0x42 => 'Bit OFF(0〜31:bit no)',
  0x43 => 'BitOnをまつ',
  0x44 => 'BitOffをまつ',
  0x45 => '32bitポート書込み',
  0x46 => 'ポートREAD=RegA'
}

$act_hash_evt = {
  0x61 => 'Set Event',
  0x62 => 'Clr Event',
  0x63 => 'Wait Event Set',
  0x64 => 'Wait Event Clr'
}

$act_hash_adc = {
  0x71 => 'A/D取込み'
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
  print "dcmotor_reg_init:#{$main_form.file_dcm}¥n"

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
  print "motor_reg_init:#{$main_form.file_ppm}¥n"

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
    cmd[8] = 0x71
  end
  $sock_port.nt_send( cmd, 'C4n2C2nNn8C2' )
end

# プロジェクト選択
class SelProject
  def initialize
    @dialog = Gtk::Dialog.new
    @dialog.set_title( 'プロジェクト選択' )
    @dialog.signal_connect( 'delete_event' ){ exit_seq(false) }
    @dialog.window_position = Gtk::Window::POS_CENTER
    @dialog.set_default_size 400, 80
    @dialog.modify_bg(Gtk::STATE_NORMAL, Gdk::Color.parse("#fffcc4"))

    # TreeView生成
    store = Gtk::ListStore.new(Integer,String)
    @treeview = Gtk::TreeView.new(store)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new('No.',renderer,'text' => 0)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new('プロジェクト名', renderer, 'text' => 1)
    @treeview.append_column(column)

    # double click
    @treeview.signal_connect("row-activated") do |view, path, column|
      exit_seq(true)
    end

    # TreeViewにプロジェクト名を載せる
    IO.readlines( PrjNames ).each_with_index do |name,i|
      iter = @treeview.model.append
      iter.set_value(0,i+1)
      iter.set_value(1,name.chop)
    end

    # 選ばれているプロジェクトを探す
    if $main_form.prj_no
      iter = @treeview.model.iter_first
      begin
        # 見つけたらカーソル移動
        if iter.get_value(0) == $main_form.prj_no
          @treeview.selection.select_iter(iter)
          break
        end
      end while iter.next!
    end

    @treeview.show_all
    @dialog.vbox.add( @treeview )

    # ダイアログを表示して戻りを処理する
    @dialog.run do |response|
      case response
      when Gtk::Dialog::RESPONSE_OK
        exit_seq(true)
      when Gtk::Dialog::RESPONSE_CLOSE
        exit_seq(false)
      end
    end
  end

  def exit_seq( choose )
    if choose == true
      str = "#{@treeview.selection.selected.get_value(0)}:#{@treeview.selection.selected.get_value(1)}"
      $main_form.enPrj.set_text( str )
      $main_form.prj_no = @treeview.selection.selected.get_value(0).to_i
      $main_form.file_tnet   = "#{Prjs[$main_form.prj_no]}/tnet#{Kakuchou_si}"
      $main_form.file_ppm    = "#{Prjs[$main_form.prj_no]}/ptable#{Kakuchou_si}"
      $main_form.file_dcm    = "#{Prjs[$main_form.prj_no]}/dctable#{Kakuchou_si}"
      $main_form.file_config = "#{Prjs[$main_form.prj_no]}/config#{Kakuchou_si}"
      $main_form.file_action = "#{Prjs[$main_form.prj_no]}/action"

      # 画面更新
      $main_form.show

      # Tnet I/O設定
#     set_tnet
      # モータレジスタの初期化
      motor_reg_init
      dcmotor_reg_init
    end
    @dialog.destroy
  end
end

# プロジェクト作成
class MkProject
  def initialize
    dialog = Gtk::Dialog.new
    dialog.set_title( 'プロジェクト作成' )
    dialog.signal_connect( 'delete_event' ){ dialog.destroy }
    dialog.window_position = Gtk::Window::POS_CENTER
    dialog.set_default_size 400, 80
    dialog.modify_bg(Gtk::STATE_NORMAL, Gdk::Color.parse("#fffcc4"))

    # 表示エリア生成
    edtLine = []
    table = Gtk::Table.new( 4, 2, false )
    ( 1..Prjs.size-1 ).each do |i|
      edtLine.push Gtk::Entry.new()
      table.attach( Gtk::Label.new( "#{i}" ),   0, 1,  i, i+1 )
      table.attach( edtLine[-1],                1, 4,  i, i+1 )
    end

    # 登録済みのプロジェクト名を表示
    if File.exist?( PrjNames )
      file = IO.readlines( PrjNames )
      edtLine.each_with_index do |x, i|
        x.set_text( file[i].chop )
      end
    end

    table.show_all
    dialog.vbox.add( table )
    dialog.add_buttons(['登録', Gtk::Dialog::RESPONSE_OK])
#   dialog.add_buttons(['閉じる', Gtk::Dialog::RESPONSE_CLOSE]) if $main_form.prj_no

    # ダイアログを表示して戻りを処理する
    dialog.run do |response|
      case response
      when Gtk::Dialog::RESPONSE_OK
        # ファイルに書き込む
        open( PrjNames, "w" ) do |f|
          edtLine.each { |x| f << "#{x.text}\n" }
        end

        # 選ばれているプロジェクト名を更新
        if $main_form.prj_no
          str = "#{$main_form.prj_no}:#{edtLine[ $main_form.prj_no - 1 ].text}"
          $main_form.enPrj.set_text( str )

        # 最初の起動なのでプロジェクト名選択へ
        else
          SelProject.new 
        end

        # 閉じる
        dialog.destroy
      when Gtk::Dialog::RESPONSE_CLOSE
        dialog.destroy
      end
    end
  end
end

class ActCopy
  def initialize( myno )
    @exit_flag = nil

    dialog = Gtk::Dialog.new
    dialog.set_title( 'コピー' )
    dialog.signal_connect( 'delete_event' ){ @exit_flag=true; dialog.destroy }
    dialog.modify_bg(Gtk::STATE_NORMAL, Gdk::Color.parse("#fffcc4"))

    tbl = Gtk::Table.new( 7, 4, false )
    lblSts     = Gtk::Label.new(' ')
    btnCopy    = Gtk::Button.new( 'コピー' )
    @cmbFProj  = Gtk::Combo.new()
    @cmbFAct   = Gtk::Combo.new()
    @cmbTAct   = Gtk::Combo.new()
    @enTAct    = Gtk::Entry.new()
    @cbSaveOpt = Gtk::CheckButton.new('上書きコピー許可')

    tbl.attach( Gtk::Label.new( 'コピー元プロジェクトの選択' ),  0, 1,  0, 1 )
    tbl.attach( @cmbFProj,                                       0, 1,  1, 2 )
    tbl.attach( Gtk::Label.new( 'コピー元プログラムの選択' ),    0, 1,  2, 3 )
    tbl.attach( @cmbFAct,                                        0, 1,  3, 4 )
    tbl.attach( Gtk::Label.new( '  --->  ' ),                    1, 2,  0, 1 )
    tbl.attach( Gtk::Label.new( 'コピー先プログラムの選択' ),    2, 3,  0, 1 )
    tbl.attach( @cmbTAct,                                        2, 3,  1, 2 )
    tbl.attach( Gtk::Label.new( 'コピー先プログラムの名称' ),    2, 3,  2, 3 )
    tbl.attach( @enTAct,                                         2, 3,  3, 4 )
    tbl.attach( @cbSaveOpt,                                      2, 3,  4, 5 )
    tbl.attach( lblSts,                                          0, 4,  5, 6 )
    tbl.attach( btnCopy,                                         0, 4,  6, 7 )

    # コピー元プロジェクト名
    ary = []
    file = IO.readlines( PrjNames )
    file.each_with_index { |p, i| ary.push( "#{i+1}:#{p.chop}" ) }
    @cmbFProj.set_popdown_strings( ary )

    # コピー先プログラム
    cfg_name = "#{Prjs[myno]}/config#{Kakuchou_si}"
    if File.exist? cfg_name
      ary = []
      file = IO.readlines( cfg_name )
      file.each_with_index { |p, i| ary.push( "#{i+1}:#{p.split(/,/)[1]}" ) }
      @cmbTAct.set_popdown_strings( ary )
    else
      @cmbTAct.set_popdown_strings( ["1:","2:","3:","4:","5:","6:","7:","8:","9:","10:",
                                  "11:","12:","13:","14:","15:","16:","17:","18:","19:","20:"] )
    end

    # プロジェクト変更
    @cmbFProj.entry.signal_connect('changed') do |widget|
      change_project if @exit_flag == nil
    end

    btnCopy.signal_connect( 'clicked' ) do |widget|
      lblSts.set_text ''
      # コピー先確認
      if "#{@enTAct.text}".size < 1
        lblSts.set_text 'コピー先プログラムの名称が未入力です'
      end
      tcp_no = @cmbTAct.entry.text.split(/:/)[0].to_i
      tcp_name = "#{Prjs[myno]}/action#{tcp_no}#{Kakuchou_si}"
      if @cbSaveOpt.active? == false && File.exist?( tcp_name )
        lblSts.set_text 'コピー先プログラムが存在します(強行は上書きコピー許可してください)'
      end
      # コピー元確認
      fcp_no1 = @cmbFProj.entry.text.split(/:/)[0].to_i
      fcp_no2 = @cmbFAct.entry.text.split(/:/)[0].to_i
      fcp_name = "#{Prjs[fcp_no1]}/action#{fcp_no2}#{Kakuchou_si}"
      if File.exist?( fcp_name ) == false
        lblSts.set_text 'コピー元プログラムが存在しません'
      end

      if "#{lblSts.text}".size < 1
        # コピー
        FileUtils.cp( fcp_name, tcp_name )

        # 名称保存
        cfg_name = "#{Prjs[myno]}/config#{Kakuchou_si}"
        file = IO.readlines( cfg_name )
        open( cfg_name, 'w' ) do |f|
          file.each do |p|
            ary = p.split(/,/)
            if ary[0].to_i != tcp_no
              f << p
            else
              ary[1] = @enTAct.text
              f << ary.join(',')
            end
          end
        end

        # 名称表示
        $main_form.name[ tcp_no-1 ].set_text @enTAct.text

        @exit_flag=true
        dialog.destroy
      end
    end

    change_project

    dialog.action_area.pack_start( tbl )
    dialog.show_all
  end

  def change_project
    /(\d*):/ =~ @cmbFProj.entry.text
    cfg_name = "#{Prjs[$1.to_i]}/config#{Kakuchou_si}"
    if File.exist? cfg_name
      file = IO.readlines( cfg_name )
      ary = []
      file.each_with_index { |p,i| ary.push( "#{i+1}:#{p.split(/,/)[1]}" ) }
      @cmbFAct.set_popdown_strings( ary )
    else
      @cmbFAct.set_popdown_strings( [""] )
    end
  end
end

class ActDelete
  def initialize( myno )
    list = []

    cfg_file = IO.readlines( $main_form.file_config ) if File.exist? $main_form.file_config

    # actionファイルの有無を調べる
    (1..20).each do |i|
      fname = "#{Prjs[myno]}/action#{i}#{Kakuchou_si}"
      if File.exist? fname
        cfg_file.each do |line|
          ary = line.split(/,/)
          if ary[0].to_i == i
            en = Gtk::Entry.new
            en.set_text "#{i}:#{ary[1]}"

            btn = Gtk::Button.new( '削除' )
            btn.signal_connect( 'clicked' ) do |widget|
              list.each do |wds|
                if wds[1] == widget

                  # ファイル除去
                  File.unlink wds[2]
                  # 名称表示
                  $main_form.name[ wds[3] ].set_text( '' )

                  @dialog.destroy
                  break
                end
              end
            end

            list.push( [ en, btn, fname, i-1 ] )
            break
          end
        end
      end
    end

    if list.size > 0
      @dialog = Gtk::Dialog.new
      @dialog.set_title( '削除' )
      @dialog.signal_connect( 'delete_event' ){ @dialog.destroy }
      @dialog.modify_bg(Gtk::STATE_NORMAL, Gdk::Color.parse("#fffcc4"))

      tbl = Gtk::Table.new( 7, 4, false )
      list.each_with_index do |widgets, i|
        tbl.attach( widgets[0],  0, 1,  i, i+1 )
        tbl.attach( widgets[1],  1, 2,  i, i+1 )
      end

      @dialog.action_area.pack_start( tbl )
      @dialog.show_all
    end
  end
end

# DCモーター情報設定
class DcmConf
  def initialize
    dialog = Gtk::Dialog.new
    dialog.set_title( 'DCモータ情報' )
    dialog.signal_connect( 'delete_event' ){ exit_seq; dialog.destroy }
    dialog.window_position = Gtk::Window::POS_CENTER
    dialog.modify_bg(Gtk::STATE_NORMAL, Gdk::Color.parse("#fffcc4"))

    # TreeView生成
    store = Gtk::ListStore.new(String,String,String,String,String,String,String,String,String,String,String,String,String,String)
    @treeview = Gtk::TreeView.new(store)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(" ノード番号 \n モータ番号 ",renderer,'text' => 0)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(' 使う ',renderer,'text' => 1)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(" Home \n Active ",renderer,'text' => 2)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(" Step \n Active ",renderer,'text' => 3)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(" Home \n OUT ",renderer,'text' => 4)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(" Home \n IN ",renderer,'text' => 5)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(' STEP ',renderer,'text' => 6)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(' HOME ',renderer,'text' => 7)
    @treeview.append_column(column)

    sw = Gtk::ScrolledWindow.new(nil,nil)
    sw.set_shadow_type(Gtk::SHADOW_ETCHED_IN)
    sw.set_policy(Gtk::POLICY_NEVER,Gtk::POLICY_AUTOMATIC)
    sw.add( @treeview )

    # Edit
    cbUse           = Gtk::CheckButton.new( '' )
    spnAdjustNNo     = Gtk::Adjustment.new( 1, 1, 64, 1, 2, 0 )
    spnBtnNNo       = Gtk::SpinButton.new( spnAdjustNNo, 0, 0 )
    spnAdjustMNo     = Gtk::Adjustment.new( 1, 1, 3, 1, 2, 0 )
    spnBtnMNo       = Gtk::SpinButton.new( spnAdjustMNo, 0, 0 )
    spnAdjustInit    = Gtk::Adjustment.new( 1, 1, 9999, 1, 2, 0 )
    spnBtnInit      = Gtk::SpinButton.new( spnAdjustInit, 0, 0 )
    spnAdjustRatio   = Gtk::Adjustment.new( 1, 1, 9999, 1, 2, 0 )
    spnBtnRatio     = Gtk::SpinButton.new( spnAdjustRatio, 0, 0 )
    spnAdjustHOPulse = Gtk::Adjustment.new( 0, 0, 9999, 1, 2, 0 )
    spnBtnHOPulse   = Gtk::SpinButton.new( spnAdjustHOPulse, 0, 0 )
    spnAdjustHIPulse = Gtk::Adjustment.new( 0, 0, 9999, 1, 2, 0 )
    spnBtnHIPulse   = Gtk::SpinButton.new( spnAdjustHIPulse, 0, 0 )
    spnAdjustHA      = Gtk::Adjustment.new( 1, 0, 1, 1, 2, 0 )
    spnBtnHA        = Gtk::SpinButton.new( spnAdjustHA, 0, 0 )
    spnAdjustSA      = Gtk::Adjustment.new( 1, 0, 1, 1, 2, 0 )
    spnBtnSA        = Gtk::SpinButton.new( spnAdjustSA, 0, 0 )
    spnAdjustHO      = Gtk::Adjustment.new( 1, 0, 1, 1, 2, 0 )
    spnBtnHO        = Gtk::SpinButton.new( spnAdjustHO, 0, 0 )
    spnAdjustHI      = Gtk::Adjustment.new( 1, 0, 1, 1, 2, 0 )
    spnBtnHI        = Gtk::SpinButton.new( spnAdjustHI, 0, 0 )
    spnAdjustStep    = Gtk::Adjustment.new( 1, 0, 1, 1, 2, 0 )
    spnBtnStep      = Gtk::SpinButton.new( spnAdjustStep, 0, 0 )
    spnAdjustHome    = Gtk::Adjustment.new( 1, 0, 1, 1, 2, 0 )
    spnBtnHome      = Gtk::SpinButton.new( spnAdjustHome, 0, 0 )
    edtR9     = Gtk::Entry.new()
    edtR10    = Gtk::Entry.new()
    edtR11    = Gtk::Entry.new()
    btnWrite  = Gtk::Button.new( '書込み' )
    btnDelete = Gtk::Button.new( '消去' )

    group11 = Gtk::Table.new( 2, 2, false )
    group11.attach( Gtk::Label.new( " ノード番号 " ),                 0,  1, 0, 1 )
    group11.attach( Gtk::Label.new( " モータ番号 " ),                 1,  2, 0, 1 )
    group11.attach( Gtk::Label.new( ' 使う ' ),                       2,  3, 0, 1 )
    group11.attach( Gtk::Label.new( " Home Active \n 0:L \n 1:H " ),  3,  4, 0, 1 )
    group11.attach( Gtk::Label.new( " STEP Active \n 0:L \n 1:H " ),  4,  5, 0, 1 )
    group11.attach( Gtk::Label.new( " Home OUT ¥\ 0:CW \n 1:CCW " ),  5,  6, 0, 1 )
    group11.attach( Gtk::Label.new( " Home IN \n 0:CW \n 1:CCW " ),   6,  7, 0, 1 )
    group11.attach( Gtk::Label.new( " STEP \n 0:CW \n 1:CCW " ),      7,  8, 0, 1 )
    group11.attach( Gtk::Label.new( " HOME \n 0:CW \n 1:CCW " ),      8,  9, 0, 1 )
    group11.attach( spnBtnNNo,                                        0,  1, 1, 2 )
    group11.attach( spnBtnMNo,                                        1,  2, 1, 2 )
    group11.attach( cbUse,                                            2,  3, 1, 2 )
    group11.attach( spnBtnHA,                                         3,  4, 1, 2 )
    group11.attach( spnBtnSA,                                         4,  5, 1, 2 )
    group11.attach( spnBtnHO,                                         5,  6, 1, 2 )
    group11.attach( spnBtnHI,                                         6,  7, 1, 2 )
    group11.attach( spnBtnStep,                                       7,  8, 1, 2 )
    group11.attach( spnBtnHome,                                       8,  9, 1, 2 )

    group13 = Gtk::HBox.new()
    group13.pack_start( btnWrite )
    group13.pack_start( btnDelete )

    table = Gtk::Table.new( 2, 2, false )
    # スクロール幅の確保
    (1..20).each { |i| table.attach( Gtk::Label.new( "  " ),  0, 1, i-1, i ) }
    table.attach( Gtk::Label.new( "  " ),  2, 3, 0,   1 )
    table.attach( sw,                      1, 2, 0,  20 )
    table.attach( group11,                 1, 2, 21, 22 )
    table.attach( group13,                 1, 2, 24, 25 )

    # TreeViewに表示
    if File.exist?( $main_form.file_dcm )
      open( $main_form.file_dcm, "r" ) do |f|
        while line = f.gets
          ary = (line.chop).split( /,/ )
          iter = @treeview.model.append
          treeview_add(ary, iter)
        end
      end
    end

    # treeview選択
    @treeview.selection.signal_connect( 'changed' ) do |widget|
      if widget.selected
        ary = widget.selected[0].split(/-/)
        spnBtnNNo.set_value( ary[0].to_i )
        spnBtnMNo.set_value( ary[1].to_i )
        cbUse.active = ( widget.selected[1] == '*' ) ? true : false;
        spnBtnHA.set_value( widget.selected[2].to_i )
        spnBtnSA.set_value( widget.selected[3].to_i )
        spnBtnHO.set_value( ( widget.selected[4] == "CW" ) ? 0 : 1 )
        spnBtnHI.set_value( ( widget.selected[5] == "CW" ) ? 0 : 1 )
        spnBtnStep.set_value( ( widget.selected[6] == "CW" ) ? 0 : 1 )
        spnBtnHome.set_value( ( widget.selected[7] == "CW" ) ? 0 : 1 )
      end
    end

    # 削除
    btnDelete.signal_connect( 'clicked' ) do |widget|
      @treeview.model.remove( @treeview.selection.selected ) if @treeview.selection.selected
    end

    # 書き込み
    btnWrite.signal_connect( 'clicked' ) do |widget|
      fmt = [ "#{spnBtnNNo.value_as_int}",
              "#{spnBtnMNo.value_as_int}",
              ( cbUse.active? ) ? '*' : '',
              "#{spnBtnHA.value_as_int}",
              "#{spnBtnSA.value_as_int}",
              ( spnBtnHO.value_as_int == 0 ) ? "CW" : "CCW",
              ( spnBtnHI.value_as_int == 0 ) ? "CW" : "CCW",
              ( spnBtnStep.value_as_int == 0 ) ? "CW" : "CCW",
              ( spnBtnHome.value_as_int == 0 ) ? "CW" : "CCW"
      ]

      flag=nil
      @treeview.model.each do |model,path,iter|
        ary   = iter.get_value(0).split(/-/)
        tline = (ary[0].to_i * 100) + ary[1].to_i
        eline = (fmt[0].to_i * 100) + fmt[1].to_i
        # 入れ替え
        if tline == eline
          treeview_add(fmt, iter)
          flag = true
          break
        # 挿入
        elsif tline > eline
          niter = @treeview.model.insert_before(iter)
          treeview_add(fmt, niter)
          flag = true
          break
        end
      end
      # 追加
      if flag == nil
        iter = @treeview.model.append
        treeview_add(fmt, iter)
      end
    end

    table.show_all
    dialog.vbox.add( table )
#   dialog.add_buttons(['閉じる', Gtk::Dialog::RESPONSE_CLOSE])

    # ダイアログを表示して戻りを処理する
    dialog.run
  end

  # DCM treeviewに追加
  def treeview_add(line, iter)
    node = ''
    line.each_with_index do |dt,i|
      if i == 0
        node = dt
      elsif i == 1
        iter.set_value(0, node + '-' + dt)
      else
        iter.set_value(i-1, dt)
      end
    end
  end

  # 終了処理
  def exit_seq
    open( $main_form.file_dcm, "w" ) do |f|
      @treeview.model.each do |model,path,iter|
        ( 0..7 ).each do |j| 
          if j == 0
            f << "#{iter[j]}".gsub(/-/, ',')
          else
            f << ",#{iter[j]}"
          end
        end
        f << "\n"
      end
    end

    # モータレジスタの初期化
    dcmotor_reg_init
  end
end

# パルスモーター情報設定
class Moter
  def initialize
    dialog = Gtk::Dialog.new
    dialog.set_title( 'パルスモータ情報' )
    dialog.signal_connect( 'delete_event' ){ exit_seq; dialog.destroy }
    dialog.window_position = Gtk::Window::POS_CENTER
    dialog.modify_bg(Gtk::STATE_NORMAL, Gdk::Color.parse("#fffcc4"))

    # TreeView生成
    store = Gtk::ListStore.new(String,String,String,String,String,String,String,String,String,String,String,String,String,String)
    @treeview = Gtk::TreeView.new(store)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(" ノード番号 \n モータ番号 ",renderer,'text' => 0)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(' 使う ',renderer,'text' => 1)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(" Init \n pulse ",renderer,'text' => 2)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(' 倍率 ',renderer,'text' => 3)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(" Home OUT \n pulse ",renderer,'text' => 4)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(" Home In \n pulse ",renderer,'text' => 5)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(" Home \n Active ",renderer,'text' => 6)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(" Home \n OUT ",renderer,'text' => 7)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(" Home \n IN ",renderer,'text' => 8)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(' STEP ',renderer,'text' => 9)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(' HOME ',renderer,'text' => 10)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(' R9 ',renderer,'text' => 11)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(' R10 ',renderer,'text' => 12)
    @treeview.append_column(column)
    renderer = Gtk::CellRendererText.new
    column   = Gtk::TreeViewColumn.new(' R11 ',renderer,'text' => 13)
    @treeview.append_column(column)

    sw = Gtk::ScrolledWindow.new(nil,nil)
    sw.set_shadow_type(Gtk::SHADOW_ETCHED_IN)
    sw.set_policy(Gtk::POLICY_NEVER,Gtk::POLICY_AUTOMATIC)
    sw.add( @treeview )

    # Edit
    cbUse           = Gtk::CheckButton.new( '' )
    spnAdjustNNo     = Gtk::Adjustment.new( 1, 1, 64, 1, 2, 0 )
    spnBtnNNo       = Gtk::SpinButton.new( spnAdjustNNo, 0, 0 )
    spnAdjustMNo     = Gtk::Adjustment.new( 1, 1, 3, 1, 2, 0 )
    spnBtnMNo       = Gtk::SpinButton.new( spnAdjustMNo, 0, 0 )
    spnAdjustInit    = Gtk::Adjustment.new( 1, 1, 9999, 1, 2, 0 )
    spnBtnInit      = Gtk::SpinButton.new( spnAdjustInit, 0, 0 )
    spnAdjustRatio   = Gtk::Adjustment.new( 1, 1, 9999, 1, 2, 0 )
    spnBtnRatio     = Gtk::SpinButton.new( spnAdjustRatio, 0, 0 )
    spnAdjustHOPulse = Gtk::Adjustment.new( 0, 0, 9999, 1, 2, 0 )
    spnBtnHOPulse   = Gtk::SpinButton.new( spnAdjustHOPulse, 0, 0 )
    spnAdjustHIPulse = Gtk::Adjustment.new( 0, 0, 9999, 1, 2, 0 )
    spnBtnHIPulse   = Gtk::SpinButton.new( spnAdjustHIPulse, 0, 0 )
    spnAdjustHA      = Gtk::Adjustment.new( 1, 0, 1, 1, 2, 0 )
    spnBtnHA        = Gtk::SpinButton.new( spnAdjustHA, 0, 0 )
    spnAdjustHO      = Gtk::Adjustment.new( 1, 0, 1, 1, 2, 0 )
    spnBtnHO        = Gtk::SpinButton.new( spnAdjustHO, 0, 0 )
    spnAdjustHI      = Gtk::Adjustment.new( 1, 0, 1, 1, 2, 0 )
    spnBtnHI        = Gtk::SpinButton.new( spnAdjustHI, 0, 0 )
    spnAdjustStep    = Gtk::Adjustment.new( 1, 0, 1, 1, 2, 0 )
    spnBtnStep      = Gtk::SpinButton.new( spnAdjustStep, 0, 0 )
    spnAdjustHome    = Gtk::Adjustment.new( 1, 0, 1, 1, 2, 0 )
    spnBtnHome      = Gtk::SpinButton.new( spnAdjustHome, 0, 0 )
    edtR9     = Gtk::Entry.new()
    edtR10    = Gtk::Entry.new()
    edtR11    = Gtk::Entry.new()
    btnWrite  = Gtk::Button.new( '書込み' )
    btnDelete = Gtk::Button.new( '消去' )

    group11 = Gtk::Table.new( 2, 2, false )
    group11.attach( Gtk::Label.new( " ノード番号 " ),                 0,  1, 0, 1 )
    group11.attach( Gtk::Label.new( " モータ番号 " ),                 1,  2, 0, 1 )
    group11.attach( Gtk::Label.new( ' 使う ' ),                       2,  3, 0, 1 )
    group11.attach( Gtk::Label.new( " Init pulse " ),                 3,  4, 0, 1 )
    group11.attach( Gtk::Label.new( ' 倍率 ' ),                       4,  5, 0, 1 )
    group11.attach( Gtk::Label.new( " Home OUT \n pulse " ),          5,  6, 0, 1 )
    group11.attach( Gtk::Label.new( " Home IN \n pulse " ),           6,  7, 0, 1 )
    group11.attach( Gtk::Label.new( " Home Active \n 0:L \n 1:H " ),  7,  8, 0, 1 )
    group11.attach( Gtk::Label.new( " Home OUT \n 0:CW \n 1:CCW " ),  8,  9, 0, 1 )
    group11.attach( Gtk::Label.new( " Home IN \n 0:CW \n 1:CCW " ),   9, 10, 0, 1 )
    group11.attach( Gtk::Label.new( " STEP \n 0:CW \n 1:CCW " ),     10, 11, 0, 1 )
    group11.attach( Gtk::Label.new( " HOME \n 0:CW \n 1:CCW " ),     11, 12, 0, 1 )
    group11.attach( spnBtnNNo,                                        0,  1, 1, 2 )
    group11.attach( spnBtnMNo,                                        1,  2, 1, 2 )
    group11.attach( cbUse,                                            2,  3, 1, 2 )
    group11.attach( spnBtnInit,                                       3,  4, 1, 2 )
    group11.attach( spnBtnRatio,                                      4,  5, 1, 2 )
    group11.attach( spnBtnHOPulse,                                    5,  6, 1, 2 )
    group11.attach( spnBtnHIPulse,                                    6,  7, 1, 2 )
    group11.attach( spnBtnHA,                                         7,  8, 1, 2 )
    group11.attach( spnBtnHO,                                         8,  9, 1, 2 )
    group11.attach( spnBtnHI,                                         9, 10, 1, 2 )
    group11.attach( spnBtnStep,                                      10, 11, 1, 2 )
    group11.attach( spnBtnHome,                                      11, 12, 1, 2 )

    group12 = Gtk::Table.new( 2, 2, false )
    group12.attach( Gtk::Label.new( "R9" ),    0, 1, 0, 1 )
    group12.attach( Gtk::Label.new( "R10" ),   1, 2, 0, 1 )
    group12.attach( Gtk::Label.new( "R11" ),   2, 3, 0, 1 )
    group12.attach( edtR9,                     0, 1, 1, 2 )
    group12.attach( edtR10,                    1, 2, 1, 2 )
    group12.attach( edtR11,                    2, 3, 1, 2 )

    group13 = Gtk::HBox.new()
    group13.pack_start( btnWrite )
    group13.pack_start( btnDelete )

    table = Gtk::Table.new( 2, 2, false )
    # スクロール幅の確保
    (1..20).each { |i| table.attach( Gtk::Label.new( "  " ),  0, 1, i-1, i ) }
    table.attach( Gtk::Label.new( "  " ),  2, 3, 0,   1 )
    table.attach( sw,                      1, 2, 0,  20 )
    table.attach( group11,                 1, 2, 21, 22 )
    table.attach( group12,                 1, 2, 22, 23 )
    table.attach( group13,                 1, 2, 24, 25 )

    # TreeViewに表示
    if File.exist?( $main_form.file_ppm )
      open( $main_form.file_ppm, "r" ) do |f|
        while line = f.gets
          ary = (line.chop).split( /,/ )
          if ary.size >= 15
            iter = @treeview.model.append
            treeview_add(ary, iter)
          end
        end
      end
    end

    # treeview選択
    @treeview.selection.signal_connect( 'changed' ) do |widget|
      if widget.selected
        ary = widget.selected[0].split(/-/)
        spnBtnNNo.set_value( ary[0].to_i )
        spnBtnMNo.set_value( ary[1].to_i )
        cbUse.active = ( widget.selected[1] == '*' ) ? true : false;
        spnBtnInit.set_value( widget.selected[2].to_i )
        spnBtnRatio.set_value( widget.selected[3].to_i )
        spnBtnHOPulse.set_value( widget.selected[4].to_i )
        spnBtnHIPulse.set_value( widget.selected[5].to_i )
        spnBtnHA.set_value( widget.selected[6].to_i )
        spnBtnHO.set_value( ( widget.selected[7] == "CW" ) ? 0 : 1 )
        spnBtnHI.set_value( ( widget.selected[8] == "CW" ) ? 0 : 1 )
        spnBtnStep.set_value( ( widget.selected[9] == "CW" ) ? 0 : 1 )
        spnBtnHome.set_value( ( widget.selected[10] == "CW" ) ? 0 : 1 )
        edtR9.set_text( "%024b" % [ "#{widget.selected[11]}".hex ] )
        edtR10.set_text( "%024b" % [ "#{widget.selected[12]}".hex ] )
        edtR11.set_text( "%024b" % [ "#{widget.selected[13]}".hex ] )
      end
    end

    # 削除
    btnDelete.signal_connect( 'clicked' ) do |widget|
      @treeview.model.remove( @treeview.selection.selected ) if @treeview.selection.selected
    end

    # 書き込み
    btnWrite.signal_connect( 'clicked' ) do |widget|
      fmt = [ "#{spnBtnNNo.value_as_int}",
              "#{spnBtnMNo.value_as_int}",
              ( cbUse.active? ) ? '*' : '',
              "#{spnBtnInit.value_as_int}",
              "#{spnBtnRatio.value_as_int}",
              "#{spnBtnHOPulse.value_as_int}",
              "#{spnBtnHIPulse.value_as_int}",
              "#{spnBtnHA.value_as_int}",
              ( spnBtnHO.value_as_int == 0 ) ? "CW" : "CCW",
              ( spnBtnHI.value_as_int == 0 ) ? "CW" : "CCW",
              ( spnBtnStep.value_as_int == 0 ) ? "CW" : "CCW",
              ( spnBtnHome.value_as_int == 0 ) ? "CW" : "CCW",
              "%x" % [ b2d( edtR9.text ) ],
              "%x" % [ b2d( edtR10.text ) ],
              "%x" % [ b2d( edtR11.text ) ]
      ]

      flag=nil
      @treeview.model.each do |model,path,iter|
        ary   = iter.get_value(0).split(/-/)
        tline = (ary[0].to_i * 100) + ary[1].to_i
        eline = (fmt[0].to_i * 100) + fmt[1].to_i
        # 入れ替え
        if tline == eline
          treeview_add(fmt, iter)
          flag = true
          break
        # 挿入
        elsif tline > eline
          niter = @treeview.model.insert_before(iter)
          treeview_add(fmt, niter)
          flag = true
          break
        end
      end
      # 追加
      if flag == nil
        iter = @treeview.model.append
        treeview_add(fmt, iter)
      end
    end

    table.show_all
    dialog.vbox.add( table )
#   dialog.add_buttons(['閉じる', Gtk::Dialog::RESPONSE_CLOSE])

    # ダイアログを表示して戻りを処理する
    dialog.run
  end

  # PM treeviewに追加
  def treeview_add(line, iter)
    node = ''
    line.each_with_index do |dt,i|
      if i == 0
        node = dt
      elsif i == 1
        iter.set_value(0, node + '-' + dt)
      else
        iter.set_value(i-1, dt)
      end
    end
  end

  # 終了処理
  def exit_seq
    open( $main_form.file_ppm, "w" ) do |f|
      @treeview.model.each do |model,path,iter|
        ( 0..13 ).each do |j| 
          if j == 0
            f << "#{iter[j]}".gsub(/-/, ',')
          else
            f << ",#{iter[j]}"
          end
        end
        f << "\n"
      end
    end

    # モータレジスタの初期化
    motor_reg_init
  end
end

# PPM微調整
class PPMAdjustPulse
  attr_reader :pulse

  def initialize(console, ary)
    @pulse = nil
    @my_console_no = console

    dialog = Gtk::Dialog.new
    dialog.set_title( 'PPM微調整' )
    dialog.signal_connect( 'delete_event' ){ dialog.destroy }
    dialog.window_position = Gtk::Window::POS_CENTER
    dialog.set_default_size 800, 200
    dialog.modify_bg(Gtk::STATE_NORMAL, Gdk::Color.parse("#fffcc4"))

    # TreeView生成
    store = Gtk::ListStore.new(String,String,String,String)
    @treeview = Gtk::TreeView.new(store)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' 行番号 ',renderer,'text' => 0)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' '*50,renderer,'text' => 1)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' 種類 ',renderer,'text' => 2)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(" ノード番号 - モータ番号 ",renderer,'text' => 3)
    @treeview.append_column(column)

    iter = @treeview.model.append
    iter.set_value(0, "#{ary[0]}")
    iter.set_value(1, "#{ary[1]}")
    iter.set_value(2, "#{ary[2]}")
    iter.set_value(3, "#{ary[3]}-#{ary[4]}")

    sw = Gtk::ScrolledWindow.new()
    sw.set_shadow_type(Gtk::SHADOW_ETCHED_IN)
    sw.set_policy(Gtk::POLICY_NEVER,Gtk::POLICY_AUTOMATIC)
    sw.add( @treeview )

    spnAdjustPulse    = Gtk::Adjustment.new( 10, 1, 9999, 1, 2, 0 )
    @spnBtnAdjustPls  = Gtk::SpinButton.new( spnAdjustPulse, 0, 0 )
    btnCCW            = Gtk::Button.new( '-パルス移動' )
    btnCW             = Gtk::Button.new( '+パルス移動' )
    adjust_func = Gtk::HBox.new()
    adjust_func.pack_start( Gtk::Label.new( '微調整' ) )
    adjust_func.pack_start( @spnBtnAdjustPls )
    adjust_func.pack_start( btnCCW )
    adjust_func.pack_start( btnCW )

    btnCCW.signal_connect( 'clicked' ) { adjust_exec(ary, '-') }
    btnCW.signal_connect( 'clicked' ) { adjust_exec(ary, '+') }

    spnNewPulse    = Gtk::Adjustment.new( ary[6].to_i, 1, 99999, 1, 2, 0 )
    @spnBtnNewPls  = Gtk::SpinButton.new( spnNewPulse, 0, 0 )
    result_func = Gtk::HBox.new()
    result_func.pack_start( Gtk::Label.new( "現在設定パルス位置: #{ary[6]}  -> 新設定パルス位置" ) )
    result_func.pack_start( @spnBtnNewPls )

    # 表示エリア生成
    table = Gtk::Table.new( 5, 5, false )
    table.attach( sw,                           0,  5,  0, 2 )
    table.attach( Gtk::Label.new( ' ' ),        0,  1,  2, 3 )
    table.attach( adjust_func,                  0,  2,  3, 4 )
    table.attach( Gtk::Label.new( ' ' ),        4,  5,  3, 4 )
    table.attach( Gtk::Label.new( ' ' ),        0,  1,  4, 5 )
    table.attach( result_func,                  0,  2,  5, 6 )
    table.attach( Gtk::Label.new( ' ' ),        0,  1,  4, 5 )

    table.show_all
    dialog.vbox.add( table )
    dialog.add_buttons(['登録', Gtk::Dialog::RESPONSE_OK])
    dialog.add_buttons(['閉じる', Gtk::Dialog::RESPONSE_CLOSE])

    @treeview.selection.mode = Gtk::SELECTION_NONE
    dialog.set_focus(@spnBtnAdjustPls)

    # ダイアログを表示して戻りを処理する
    dialog.run do |response|
      case response
      when Gtk::Dialog::RESPONSE_OK
        @pulse = @spnBtnNewPls.value_as_int
        # 閉じる
        dialog.destroy
      when Gtk::Dialog::RESPONSE_CLOSE
        dialog.destroy
      end
    end
  end

  def adjust_exec(ary, arg)
    # Actionが停止であることを確認する
    return if $main_form.status[ @my_console_no-1 ].text != "stop"

    if $sock_port.open_err == nil
      # ActionプロセスへREADY要求
      $sock_port.nt_send( [STX, 0x07, 0x00, 0x00, 0xC014, @my_console_no, 0x00, ETX], 'C4nC3' )

      # Actionプロセスへ動作情報送信
      if arg == '+'
        ary[5] = $act_hash_ppm[0x12]
        ary[6] = "#{@spnBtnAdjustPls.value_as_int}"
        action_info_send(@my_console_no, ary)
      else
        ary[5] = $act_hash_ppm[0x13]
        ary[6] = "#{@spnBtnAdjustPls.value_as_int}"
        action_info_send(@my_console_no, ary)
      end

      # ActionプロセスへSTART要求
      $main_form.status[ @my_console_no-1 ].set_text "run"
      $sock_port.nt_send( [STX, 0x11, 0x00, 0x00, 0xC015, @my_console_no, 1, 0, 0, 0, 0, ETX], 'C4nCNn3C2' )
    end

    if arg == '+'
      @spnBtnNewPls.set_value( @spnBtnNewPls.value_as_int + @spnBtnAdjustPls.value_as_int )
    else
      @spnBtnNewPls.set_value( @spnBtnNewPls.value_as_int - @spnBtnAdjustPls.value_as_int )
    end
  end
end

# 動作プログラムの入力
class Input

  attr_accessor :lblStatus, :treeview

  def initialize( arg )
    @my_console_no = arg.to_i
    @clist_change=nil

    dialog = Gtk::Dialog.new
    dialog.set_title( 'Action入力部' )
    dialog.signal_connect( 'delete_event' ){ exit_seq; dialog.destroy }
    dialog.window_position = Gtk::Window::POS_CENTER

    # Title
    @edtTitle     = Gtk::Entry.new()
    btnComReset   = Gtk::Button.new( "COMポートリセット" )

    # START Function new
    spnAdjustTimes = Gtk::Adjustment.new( 1, 0, 999999, 1, 2, 0 )
    @spnBtnTimes   = Gtk::SpinButton.new( spnAdjustTimes, 0, 0 )
    btnGo          = Gtk::Button.new( '  Go.  ' )
    btnStop        = Gtk::Button.new( ' STOP ' )

    btnStep           = Gtk::Button.new( '次' )
    btnMore           = Gtk::Button.new( 'もう一度' )

    btnAdjustStart    = Gtk::Button.new( 'PPM微調整' )

    # START Function 配置
    start_func = Gtk::HBox.new()
    start_func.pack_start( Gtk::Label.new( '繰返し回数' ) )
    start_func.pack_start( @spnBtnTimes )
    start_func.pack_start( btnGo )
    start_func.pack_start( btnStop )
    start_func.pack_start( Gtk::Label.new( '     ' ) )
    start_func.pack_start( btnStep )
    start_func.pack_start( btnMore )
    start_func.pack_start( Gtk::Label.new( '     ' ) )
    start_func.pack_start( btnAdjustStart )
    start_func.pack_start( Gtk::Label.new( '         ' ) )

    # TreeView生成
    store = Gtk::ListStore.new(String,String,String,String,String,String,String,String,String,String,String,String,String,String)
    @treeview = Gtk::TreeView.new(store)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' 行番号 ',renderer,'text' => 0)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' '*50,renderer,'text' => 1)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' 種類 ',renderer,'text' => 2)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(" ノード番号 \n モータ番号 ",renderer,'text' => 3)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' Action ',renderer,'text' => 4)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(" 移動量 \n msec \n Bit ",renderer,'text' => 5)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(" 自起動 \n\n Mask ",renderer,'text' => 6)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(" 最高速 \n\n Value ",renderer,'text' => 7)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' 加速 ',renderer,'text' => 8)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' 減速 ',renderer,'text' => 9)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' 倍率 ',renderer,'text' => 10)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' 減速開始 ',renderer,'text' => 11)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' 加速S ',renderer,'text' => 12)
    @treeview.append_column(column)
    renderer  = Gtk::CellRendererText.new
    column    = Gtk::TreeViewColumn.new(' 減速S ',renderer,'text' => 13)
    @treeview.append_column(column)

    style = dialog.style
    style.font_desc = Pango::FontDescription.new("Monospace 12")
#   @treeview.style = style
    dialog.style = style

    # Shiftキーは選択した場所と現在フォーカスしている場所までの間を選択する
    @treeview.selection.mode = Gtk::SELECTION_MULTIPLE

    sw = Gtk::ScrolledWindow.new()
    sw.set_shadow_type(Gtk::SHADOW_ETCHED_IN)
    sw.set_policy(Gtk::POLICY_NEVER,Gtk::POLICY_AUTOMATIC)
    sw.add( @treeview )

    # Status new
    @lblStatus = []
    @lblStatus[0] = Gtk::Label.new( "" )
    @lblStatus[1] = Gtk::Label.new( "" )
    @lblStatus[2] = Gtk::Label.new( "" )

    # Status 配置
    group_sts = Gtk::VBox.new()
    group_sts.pack_start( @lblStatus[0] )
    group_sts.pack_start( @lblStatus[1] )
    group_sts.pack_start( @lblStatus[2] )

    # Edit PPM new
    @edtPComment     = Gtk::Entry.new()
    spnAdjustNode    = Gtk::Adjustment.new( 1, 1, 64, 1, 2, 0 )
    @spnBtnNode      = Gtk::SpinButton.new( spnAdjustNode, 0, 0 )
    spnAdjustPMNo    = Gtk::Adjustment.new( 1, 1, 64, 1, 2, 0 )
    @spnBtnPMNo      = Gtk::SpinButton.new( spnAdjustPMNo, 0, 0 )
    @cmbPAction      = Gtk::Combo.new()
    spnAdjustPPulse  = Gtk::Adjustment.new( 0, -99999, 99999, 1, 2, 0 )
    @spnBtnPPulse    = Gtk::SpinButton.new( spnAdjustPPulse, 0, 0 )
    spnAdjustPStart  = Gtk::Adjustment.new( 1, 0, 99999, 1, 2, 0 )
    @spnBtnPStart    = Gtk::SpinButton.new( spnAdjustPStart, 0, 0 )
    spnAdjustPMax    = Gtk::Adjustment.new( 1, 0, 99999, 1, 2, 0 )
    @spnBtnPMax      = Gtk::SpinButton.new( spnAdjustPMax, 0, 0 )
    spnAdjustPUSlope = Gtk::Adjustment.new( 1, 0, 99999, 1, 2, 0 )
    @spnBtnPUSlope   = Gtk::SpinButton.new( spnAdjustPUSlope, 0, 0 )
    spnAdjustPDSlope = Gtk::Adjustment.new( 1, 0, 99999, 1, 2, 0 )
    @spnBtnPDSlope   = Gtk::SpinButton.new( spnAdjustPDSlope, 0, 0 )
    spnAdjustPRatio  = Gtk::Adjustment.new( 1, 1, 99999, 1, 2, 0 )
    @spnBtnPRatio    = Gtk::SpinButton.new( spnAdjustPRatio, 0, 0 )
    spnAdjustR6      = Gtk::Adjustment.new( 0, 0, 99999, 1, 2, 0 )
    @spnBtnR6        = Gtk::SpinButton.new( spnAdjustR6, 0, 0 )
    spnAdjustR7      = Gtk::Adjustment.new( 0, 0, 99999, 1, 2, 0 )
    @spnBtnR7        = Gtk::SpinButton.new( spnAdjustR7, 0, 0 )
    spnAdjustR8      = Gtk::Adjustment.new( 0, 0, 99999, 1, 2, 0 )
    @spnBtnR8        = Gtk::SpinButton.new( spnAdjustR8, 0, 0 )
    btnPWrite        = Gtk::Button.new( '書込み' )

    # Edit PPM 配置
    table4_1 = Gtk::Table.new( 3, 4, false )
    table4_1.attach( Gtk::Label.new( CommentTitle ), 0,  8, 0, 1 )
    table4_1.attach( Gtk::Label.new( "Action" ),     8, 11, 0, 1 )
    table4_1.attach( @edtPComment,                   0,  8, 1, 2 )
    table4_1.attach( @cmbPAction,                    8, 11, 1, 2 )

    table4_1.attach( Gtk::Label.new( 'ノード番号' ), 0,  1, 2, 3 )
    table4_1.attach( Gtk::Label.new( 'モータ番号' ), 1,  2, 2, 3 )
    table4_1.attach( Gtk::Label.new( '移動量:R0' ),  2,  3, 2, 3 )
    table4_1.attach( Gtk::Label.new( '自起動:R1' ),  3,  4, 2, 3 )
    table4_1.attach( Gtk::Label.new( '最高速:R2' ),  4,  5, 2, 3 )
    table4_1.attach( Gtk::Label.new( '加速:R3 ' ),   5,  6, 2, 3 )
    table4_1.attach( Gtk::Label.new( '減速:R4 ' ),   6,  7, 2, 3 )
    table4_1.attach( Gtk::Label.new( '倍率:R5 ' ),   7,  8, 2, 3 )
    table4_1.attach( Gtk::Label.new( '減速開:R6'),   8,  9, 2, 3 )
    table4_1.attach( Gtk::Label.new( '加速S:R7' ),   9, 10, 2, 3 )
    table4_1.attach( Gtk::Label.new( '減速S:R8' ),  10, 11, 2, 3 )

    table4_1.attach( @spnBtnNode,                    0,  1, 3, 4 )
    table4_1.attach( @spnBtnPMNo,                    1,  2, 3, 4 )
    table4_1.attach( @spnBtnPPulse,                  2,  3, 3, 4 )
    table4_1.attach( @spnBtnPStart,                  3,  4, 3, 4 )
    table4_1.attach( @spnBtnPMax,                    4,  5, 3, 4 )
    table4_1.attach( @spnBtnPUSlope,                 5,  6, 3, 4 )
    table4_1.attach( @spnBtnPDSlope,                 6,  7, 3, 4 )
    table4_1.attach( @spnBtnPRatio,                  7,  8, 3, 4 )
    table4_1.attach( @spnBtnR6,                      8,  9, 3, 4 )
    table4_1.attach( @spnBtnR7,                      9, 10, 3, 4 )
    table4_1.attach( @spnBtnR8,                     10, 11, 3, 4 )
    table4_1.attach( btnPWrite,                      0, 11, 4, 5 )

    # Edit DCM new
    @edtAComment     = Gtk::Entry.new()
    spnAdjustANode   = Gtk::Adjustment.new( 1, 1, 64, 1, 2, 0 )
    @spnBtnANode     = Gtk::SpinButton.new( spnAdjustANode, 0, 0 )
    spnAdjustAPort   = Gtk::Adjustment.new( 1, 1, 64, 1, 2, 0 )
    @spnBtnAPort     = Gtk::SpinButton.new( spnAdjustAPort, 0, 0 )
    @cmbAAction      = Gtk::Combo.new()
    spnAdjustAAdd    = Gtk::Adjustment.new( 1, 0, 500, 1, 2, 0 )
    @spnBtnAAdd      = Gtk::SpinButton.new( spnAdjustAAdd, 0, 0 )
    btnAWrite        = Gtk::Button.new( '書込み' )

    # Edit DCM 配置
    table4_2 = Gtk::Table.new( 3, 4, false )
    table4_2.attach( Gtk::Label.new( CommentTitle ),          0, 5, 0, 1 )
    table4_2.attach( @edtAComment,                            0, 6, 1, 2 )
    table4_2.attach( Gtk::Label.new( "ノード番号" ),          0, 1, 2, 3 )
    table4_2.attach( Gtk::Label.new( "ポート番号" ),          1, 2, 2, 3 )
    table4_2.attach( Gtk::Label.new( "Action" ),              2, 4, 2, 3 )
    table4_2.attach( Gtk::Label.new( "移動量/突入量(msec)" ), 4, 6, 2, 3 )
    table4_2.attach( @spnBtnANode,                            0, 1, 3, 4 )
    table4_2.attach( @spnBtnAPort,                            1, 2, 3, 4 )
    table4_2.attach( @cmbAAction,                             2, 4, 3, 4 )
    table4_2.attach( @spnBtnAAdd,                             4, 6, 3, 4 )
    table4_2.attach( btnAWrite,                               0, 6, 4, 5 )

    # Edit DIO new
    @edtDComment     = Gtk::Entry.new()
    spnAdjustDNode   = Gtk::Adjustment.new( 1, 1, 64, 1, 2, 0 )
    @spnBtnDNode     = Gtk::SpinButton.new( spnAdjustDNode, 0, 0 )
    spnAdjustDPort   = Gtk::Adjustment.new( 1, 1, 64, 1, 2, 0 )
    @spnBtnDPort     = Gtk::SpinButton.new( spnAdjustDPort, 0, 0 )
    @cmbDAction      = Gtk::Combo.new()
    @edtDBitNo       = Gtk::Entry.new()
    btnDWrite        = Gtk::Button.new( '書込み' )

    # Edit DIO 配置
    table4_3 = Gtk::Table.new( 3, 4, false )
    table4_3.attach( Gtk::Label.new( CommentTitle ),               0, 5, 0, 1 )
    table4_3.attach( @edtDComment,                                 0, 6, 1, 2 )
    table4_3.attach( Gtk::Label.new( "ノード番号" ),               0, 1, 2, 3 )
    table4_3.attach( Gtk::Label.new( "ポート番号" ),               1, 2, 2, 3 )
    table4_3.attach( Gtk::Label.new( "Action" ),                   2, 4, 2, 3 )
    table4_3.attach( Gtk::Label.new( "BitNo.(b0-31)/0xXXXXXXXX" ), 4, 6, 2, 3 )
    table4_3.attach( @spnBtnDNode,                                 0, 1, 3, 4 )
    table4_3.attach( @spnBtnDPort,                                 1, 2, 3, 4 )
    table4_3.attach( @cmbDAction,                                  2, 4, 3, 4 )
    table4_3.attach( @edtDBitNo,                                   4, 6, 3, 4 )
    table4_3.attach( btnDWrite,                                    0, 6, 4, 5 )

    # Edit Wait new
    @edtWComment  = Gtk::Entry.new()
    spnWTimes     = Gtk::Adjustment.new( 1, 1, 99999, 1, 2, 0 )
    @spnBtnWTimes = Gtk::SpinButton.new( spnWTimes, 0, 0 )
    btnWWrite     = Gtk::Button.new( '書込み' )

    # Edit if new
    @edtIComment  = Gtk::Entry.new()
    @edtIMask     = Gtk::Entry.new()
    @edtIValue    = Gtk::Entry.new()
    spnIGoto      = Gtk::Adjustment.new( 1, 1, 99999, 1, 2, 0 )
    @spnBtnIGoto  = Gtk::SpinButton.new( spnIGoto, 0, 0 )
    btnIWrite     = Gtk::Button.new( '書込み' )

    # Edit if not new
    @edtUComment  = Gtk::Entry.new()
    @edtUMask     = Gtk::Entry.new()
    @edtUValue    = Gtk::Entry.new()
    spnUGoto      = Gtk::Adjustment.new( 1, 1, 99999, 1, 2, 0 )
    @spnBtnUGoto  = Gtk::SpinButton.new( spnUGoto, 0, 0 )
    btnUWrite     = Gtk::Button.new( '書込み' )

    # Edit Goto new
    @edtGComment  = Gtk::Entry.new()
    @cbGotoEnd    = Gtk::CheckButton.new( '最終行へJump' )
    spnGoto       = Gtk::Adjustment.new( 1, 1, 99999, 1, 2, 0 )
    @spnBtnGoto   = Gtk::SpinButton.new( spnGoto, 0, 0 )
    btnGWrite     = Gtk::Button.new( '書込み' )

    # Edit Error new
    @edtErComment   = Gtk::Entry.new()
    spnAdjustErNode = Gtk::Adjustment.new( 1, 1, 64, 1, 2, 0 )
    @spnBtnErNode   = Gtk::SpinButton.new( spnAdjustErNode, 0, 0 )
    spnAdjustErPMNo = Gtk::Adjustment.new( 1, 1, 64, 1, 2, 0 )
    @spnBtnErPort   = Gtk::SpinButton.new( spnAdjustErPMNo, 0, 0 )
    @edtErBitNo     = Gtk::Entry.new()
    btnErWrite      = Gtk::Button.new( '書込み' )

    # Edit Wait 配置
    table4_41 = Gtk::Table.new( 3, 4, false )
    table4_41.attach( Gtk::Label.new( CommentTitle ), 0, 3, 0, 1 )
    table4_41.attach( @edtWComment,                   0, 3, 1, 2 )
    table4_41.attach( Gtk::Label.new( "msec." ),      3, 4, 0, 1 )
    table4_41.attach( @spnBtnWTimes,                  3, 4, 1, 2 )
    table4_41.attach( btnWWrite,                      0, 4, 3, 4 )

    # Edit if 配置
    table4_42 = Gtk::Table.new( 3, 4, false )
    table4_42.attach( Gtk::Label.new( CommentTitle ),        0, 1, 0, 1 )
    table4_42.attach( Gtk::Label.new( IFCmt ),               1, 3, 0, 1 )
    table4_42.attach( @edtIComment,                          0, 3, 1, 2 )
    table4_42.attach( Gtk::Label.new( "Mask(0xXXXX)" ),      3, 4, 0, 1 )
    table4_42.attach( @edtIMask,                             3, 4, 1, 2 )
    table4_42.attach( Gtk::Label.new( "Value(0xXXXX)" ),     4, 5, 0, 1 )
    table4_42.attach( @edtIValue,                            4, 5, 1, 2 )
    table4_42.attach( Gtk::Label.new( "goto line" ),         5, 6, 0, 1 )
    table4_42.attach( @spnBtnIGoto,                          5, 6, 1, 2 )
    table4_42.attach( btnIWrite,                             0, 6, 3, 4 )

    # Edit if not 配置
    table4_43 = Gtk::Table.new( 3, 4, false )
    table4_43.attach( Gtk::Label.new( CommentTitle ),        0, 1, 0, 1 )
    table4_43.attach( Gtk::Label.new( UNCmt ),               1, 3, 0, 1 )
    table4_43.attach( @edtUComment,                          0, 3, 1, 2 )
    table4_43.attach( Gtk::Label.new( "Mask(0xXXXX)" ),      3, 4, 0, 1 )
    table4_43.attach( @edtUMask,                             3, 4, 1, 2 )
    table4_43.attach( Gtk::Label.new( "Value(0xXXXX)" ),     4, 5, 0, 1 )
    table4_43.attach( @edtUValue,                            4, 5, 1, 2 )
    table4_43.attach( Gtk::Label.new( "goto line" ),         5, 6, 0, 1 )
    table4_43.attach( @spnBtnUGoto,                          5, 6, 1, 2 )
    table4_43.attach( btnUWrite,                             0, 6, 3, 4 )

    # Edit goto 配置
    table4_44 = Gtk::Table.new( 3, 4, false )
    table4_44.attach( Gtk::Label.new( CommentTitle ),        0, 1, 0, 1 )
    table4_44.attach( Gtk::Label.new( ' ' * 20 ),            1, 2, 0, 1 )
    table4_44.attach( @cbGotoEnd,                            2, 3, 0, 1 )
    table4_44.attach( @edtGComment,                          0, 3, 1, 2 )
    table4_44.attach( Gtk::Label.new( 'goto line' ),         5, 6, 0, 1 )
    table4_44.attach( @spnBtnGoto,                           5, 6, 1, 2 )
    table4_44.attach( btnGWrite,                             0, 6, 3, 4 )

    # Edit error 配置
    table4_45 = Gtk::Table.new( 3, 4, false )
    table4_45.attach( Gtk::Label.new( CommentTitle ),        0, 3, 0, 1 )
    table4_45.attach( Gtk::Label.new( "ノード番号" ),        3, 4, 0, 1 )
    table4_45.attach( Gtk::Label.new( "ポート番号" ),        4, 5, 0, 1 )
    table4_45.attach( Gtk::Label.new( "BitNo.(b0-31)" ),     5, 6, 0, 1 )
    table4_45.attach( @edtErComment,                         0, 3, 1, 2 )
    table4_45.attach( @spnBtnErNode,                         3, 4, 1, 2 )
    table4_45.attach( @spnBtnErPort,                         4, 5, 1, 2 )
    table4_45.attach( @edtErBitNo,                           5, 6, 1, 2 )
    table4_45.attach( btnErWrite,                            0, 6, 3, 4 )

    # Edit 命令文 配置
    @nbook4_4 = Gtk::Notebook.new()
    @nbook4_4.append_page( table4_41,  Gtk::Label.new( ' Wait ' ) )
    @nbook4_4.append_page( table4_42,  Gtk::Label.new( ' if ' ) )
    @nbook4_4.append_page( table4_43,  Gtk::Label.new( ' if not ' ) )
    @nbook4_4.append_page( table4_44,  Gtk::Label.new( ' goto ' ) )
    @nbook4_4.append_page( table4_45,  Gtk::Label.new( ' error ' ) )

    # Edit SIO new
    @edtSComment  = Gtk::Entry.new()
    spnAdjustSCh  = Gtk::Adjustment.new( 1, 1, 64, 1, 2, 0 )
    @spnBtnSCh    = Gtk::SpinButton.new( spnAdjustSCh, 0, 0 )
    @edtSText     = Gtk::Entry.new()
    @cbCr         = Gtk::CheckButton.new( 'CR' )
    btnSWrite     = Gtk::Button.new( '書込み' )

    # Edit SIO 配置
    table4_5 = Gtk::Table.new( 3, 4, false )
    table4_5.attach( Gtk::Label.new( CommentTitle ), 0, 5, 0, 1 )
    table4_5.attach( @edtSComment,                   0, 5, 1, 2 )
    table4_5.attach( Gtk::Label.new( "Chan No." ),   0, 1, 2, 3 )
    table4_5.attach( Gtk::Label.new( "Send Text" ),  1, 4, 2, 3 )
    table4_5.attach( @spnBtnSCh,                     0, 1, 3, 4 )
    table4_5.attach( @edtSText,                      1, 4, 3, 4 )
    table4_5.attach( @cbCr,                          4, 5, 3, 4 )
    table4_5.attach( btnSWrite,                      0, 5, 4, 5 )

    # Edit Event new
    @edtEComment = Gtk::Entry.new()
    @cmbEAction  = Gtk::Combo.new()
    spnAdjustENo = Gtk::Adjustment.new( 1, 1, 20, 1, 2, 0 )
    @spnBtnENo   = Gtk::SpinButton.new( spnAdjustENo, 0, 0 )
    btnEWrite    = Gtk::Button.new( '書込み' )

    # Edit Event 配置
    table4_6 = Gtk::Table.new( 3, 4, false )
    table4_6.attach( Gtk::Label.new( CommentTitle ), 0, 4, 0, 1 )
    table4_6.attach( @edtEComment,                   0, 4, 1, 2 )
    table4_6.attach( Gtk::Label.new( "Action" ),     0, 3, 2, 3 )
    table4_6.attach( Gtk::Label.new( "No." ),        3, 4, 2, 3 )
    table4_6.attach( @cmbEAction,                    0, 3, 3, 4 )
    table4_6.attach( @spnBtnENo,                     3, 4, 3, 4 )
    table4_6.attach( btnEWrite,                      0, 4, 4, 5 )

    # Edit A/D new
    @edtMComment    = Gtk::Entry.new()
    @cmbMAction     = Gtk::Combo.new()
    spnAdjustMTimes = Gtk::Adjustment.new( 1, 1, 2000, 1, 2, 0 )
    @spnBtnMTimes   = Gtk::SpinButton.new( spnAdjustMTimes, 0, 0 )
    btnMWrite       = Gtk::Button.new( '書込み' )

    # Edit A/D 配置
    table4_7 = Gtk::Table.new( 3, 4, false )
    table4_7.attach( Gtk::Label.new( CommentTitle ),   0, 4, 0, 1 )
    table4_7.attach( @edtMComment,                     0, 4, 1, 2 )
    table4_7.attach( Gtk::Label.new( "Action" ),       0, 3, 2, 3 )
    table4_7.attach( Gtk::Label.new( "msec." ),        3, 4, 2, 3 )
    table4_7.attach( @cmbMAction,                      0, 3, 3, 4 )
    table4_7.attach( @spnBtnMTimes,                    3, 4, 3, 4 )
    table4_7.attach( btnMWrite,                        0, 4, 4, 5 )

    # Notebook
    @nbook = Gtk::Notebook.new()
    @nbook.append_page( table4_1,  Gtk::Label.new( ' PPM ' ) )
    @nbook.append_page( table4_2,  Gtk::Label.new( ' DCM ' ) )
    @nbook.append_page( table4_3,  Gtk::Label.new( ' DIO ' ) )
    @nbook.append_page( @nbook4_4, Gtk::Label.new( ' 命令文 ' ) )
    @nbook.append_page( table4_5,  Gtk::Label.new( ' SIO ' ) )
    @nbook.append_page( table4_6,  Gtk::Label.new( ' Event ' ) )
    @nbook.append_page( table4_7,  Gtk::Label.new( ' A/D ' ) )

    # Edit Line new
    spnAdjustLine = Gtk::Adjustment.new( 1, 1, 9999, 1, 2, 0 )
    @spnBtnLine   = Gtk::SpinButton.new( spnAdjustLine, 0, 0 )
    btnReNo       = Gtk::Button.new( '行番号再割振り' )
    btnDelete     = Gtk::Button.new( '行削除' )

    # Edit Line 配置
    group_edit = Gtk::HBox.new()
    group_edit.pack_start( Gtk::Label.new( '行番号' ) )
    group_edit.pack_start( @spnBtnLine )
    group_edit.pack_start( Gtk::Label.new( ' ' * 10 ) )
    group_edit.pack_start( btnReNo )
    group_edit.pack_start( btnDelete )

    # 全体配置
msg = '※実行範囲は、開始行をクリックし、終了行はShiftを押しながらクリックします。'
    table = Gtk::Table.new( 3, 2, false )
    table.attach( Gtk::Label.new( 'タイトル' ),   0,  1,  0,  1 )
    table.attach( Gtk::Label.new( '実行' ),       0,  1,  1,  2 )
    table.attach( Gtk::Label.new( '    ' ),      15, 16,  1,  2 )
    table.attach( Gtk::Label.new( ' ' ),          0,  1,  2,  3 )
    table.attach( Gtk::Label.new( '編集' ),       0,  1, 25, 26 )
    # スクロール幅の確保
    (1..20).each { |i| table.attach( Gtk::Label.new( " " ),  0, 1, i+2, i+3 ) }
    table.attach( @edtTitle,               1,  7,  0,  1 )
#   table.attach( btnComReset,            13, 15,  0,  1 )
    table.attach( start_func,              1, 11,  1,  2 )
    table.attach( Gtk::Label.new( msg ),   1,  6,  3,  4 )
    table.attach( sw,                      1, 15,  4, 24 )
    table.attach( group_sts,               1, 15, 24, 25 )
    table.attach( group_edit,              1,  3, 25, 26 )
    table.attach( Gtk::Label.new( ' ' ),   0,  1, 26, 27 )
    table.attach( @nbook,                  1, 15, 27, 28 )


    # treeview click
    @treeview.signal_connect( 'cursor-changed' ) do |widget|
      widget.selection.selected_each do |model, path, iter|
        clist_clicked( iter )
        break
      end
    end

    btnDelete.signal_connect( 'clicked' ){ delete_clicked }
    btnReNo.signal_connect( 'clicked' ){ reno_clicked }
    btnPWrite.signal_connect( 'clicked' ){ ppm_write_clicked }
    btnAWrite.signal_connect( 'clicked' ){ dcm_write_clicked }
    btnDWrite.signal_connect( 'clicked' ){ dio1_write_clicked }
    btnSWrite.signal_connect( 'clicked' ){ sio_write_clicked }
    btnWWrite.signal_connect( 'clicked' ){ wait_write_clicked }
    btnIWrite.signal_connect( 'clicked' ){ if_write_clicked }
    btnUWrite.signal_connect( 'clicked' ){ unless_write_clicked }
    btnGWrite.signal_connect( 'clicked' ){ goto_write_clicked }
    btnErWrite.signal_connect( 'clicked' ){ error_write_clicked }
    btnEWrite.signal_connect( 'clicked' ){ event_write_clicked }
    btnMWrite.signal_connect( 'clicked' ){ meas_write_clicked }
    btnGo.signal_connect( 'clicked' ){ go_clicked }
    btnStop.signal_connect( 'clicked' ){ stop_clicked }
    btnStep.signal_connect( 'clicked' ){ step_clicked }
    btnMore.signal_connect( 'clicked' ){ more_clicked }
    btnAdjustStart.signal_connect( 'clicked' ){ ppm_adjust_clicked }
#   btnComReset.signal_connect( 'clicked' ){ $sock_port.reopen }

    # Combo内容
    @cmbPAction.set_popdown_strings( $act_hash_ppm.values )
    @cmbAAction.set_popdown_strings( $act_hash_dcm.values )
    @cmbDAction.set_popdown_strings( $act_hash_dio.values )
    @cmbEAction.set_popdown_strings( $act_hash_evt.values )
    @cmbMAction.set_popdown_strings( $act_hash_adc.values )

    # コラムリストに表示
    fname = $main_form.file_action + "#{@my_console_no}" + Kakuchou_si
    if File.exist?( fname )
      open( fname, "r" ) do |f|
        while line = f.gets
          ary = (line.chop).split( /,/ )
          while ary.size < 15 do
            ary.push '-'
          end
          iter = @treeview.model.append
          treeview_add(ary, iter)
        end
      end
    end

    # title
    @edtTitle.set_text( $main_form.name[ @my_console_no-1 ].text )

    dialog.set_focus(@treeview)
    table.show_all
    dialog.vbox.add( table )
#   dialog.add_buttons(['閉じる', Gtk::Dialog::RESPONSE_CLOSE])

    dialog.signal_connect("response") do |widget, response|
      exit_seq
      dialog.destroy
    end

    dialog.show_all

    return dialog
  end

  # 閉じるシーケンス
  def exit_seq
    # ファイルへ書き込む
    save_to_action_file

    # 名称保存
    if File.exist? ( $main_form.file_config )
      file = IO.readlines( $main_form.file_config )
      open( $main_form.file_config, 'w' ) do |f|
        file.each do |p|
          ary = p.split(/,/)
          if ary[0].to_i != @my_console_no
            f << p
          else
            ary[1] = @edtTitle.text
            f << ary.join(',')
          end
        end
      end
    end

    # main formへ終了通知
    $main_form.console_opened[ @my_console_no ] = nil
  end

  # コラムリストクリック
  def clist_clicked( iter )
    # Comment
    @edtPComment.set_text( iter.get_value(1) )
    @edtAComment.set_text( iter.get_value(1) )
    @edtDComment.set_text( iter.get_value(1) )
    @edtSComment.set_text( iter.get_value(1) )
    @edtWComment.set_text( iter.get_value(1) )
    @edtIComment.set_text( iter.get_value(1) )
    @edtUComment.set_text( iter.get_value(1) )
    @edtGComment.set_text( iter.get_value(1) )
    @edtErComment.set_text( iter.get_value(1) )
    @edtEComment.set_text( iter.get_value(1) )
    @edtMComment.set_text( iter.get_value(1) )

    case iter.get_value(2)
    when "PPM"
      ary = iter.get_value(3).split(/-/)
      @spnBtnNode.set_value( ary[0].to_i )
      @spnBtnPMNo.set_value( ary[1].to_i )
      @cmbPAction.entry.set_text( iter.get_value(4) )
      @spnBtnPPulse.set_value( iter.get_value(5).to_i )
      @spnBtnPStart.set_value( iter.get_value(6).to_i )
      @spnBtnPMax.set_value( iter.get_value(7).to_i )
      @spnBtnPUSlope.set_value( iter.get_value(8).to_i )
      @spnBtnPDSlope.set_value( iter.get_value(9).to_i )
      @spnBtnPRatio.set_value( iter.get_value(10).to_i )
      @spnBtnR6.set_value( iter.get_value(11).to_i )
      @spnBtnR7.set_value( iter.get_value(12).to_i )
      @spnBtnR8.set_value( iter.get_value(13).to_i )
      @nbook.set_page( 0 )
    when "DCM"
      ary = iter.get_value(3).split(/-/)
      @spnBtnANode.set_value( ary[0].to_i )
      @spnBtnAPort.set_value( ary[1].to_i )
      @cmbAAction.entry.set_text( iter.get_value(4) )
      @spnBtnAAdd.set_text( iter.get_value(5) )
      @nbook.set_page( 1 )
    when "DIO"
      ary = iter.get_value(3).split(/-/)
      @spnBtnDNode.set_value( ary[0].to_i )
      @spnBtnDPort.set_value( ary[1].to_i )
      @cmbDAction.entry.set_text( iter.get_value(4) )
      @edtDBitNo.set_text( iter.get_value(5) )
      @nbook.set_page( 2 )
    when "WAIT"
      @spnBtnWTimes.set_value( iter.get_value(5).to_i )
      @nbook.set_page( 3 )
      @nbook4_4.set_page( 0 )
    when "IF"
      @spnBtnIGoto.set_value( iter.get_value(5).to_i )
      @edtIMask.set_text( iter.get_value(6) )
      @edtIValue.set_text( iter.get_value(7) )
      @nbook.set_page( 3 )
      @nbook4_4.set_page( 1 )
    when "UNLESS"
      @spnBtnUGoto.set_value( iter.get_value(5).to_i )
      @edtUMask.set_text( iter.get_value(6) )
      @edtUValue.set_text( iter.get_value(7) )
      @nbook.set_page( 3 )
      @nbook4_4.set_page( 2 )
    when "GOTO"
      @cbGotoEnd.active  = (iter.get_value(4)=='GOTO_END') ? true : false
      @spnBtnGoto.set_value( iter.get_value(5).to_i )
      @nbook.set_page( 3 )
      @nbook4_4.set_page( 3 )
    when "ERROR"
      @nbook.set_page( 3 )
      ary = iter.get_value(3).split(/-/)
      @spnBtnErNode.set_value( ary[0].to_i )
      @spnBtnErPort.set_value( ary[1].to_i )
      @edtErBitNo.set_text( iter.get_value(5) )
      @nbook4_4.set_page( 4 )
    when "SIO"
      @spnBtnSCh.set_value( iter.get_value(3).to_i )
      @cbCr.active = ( iter.get_value(4) == 'SIO ADD CR' ) ? true : false
      @edtSText.set_text( iter.get_value(5) )
      @nbook.set_page( 4 )
    when "EVENT"
      @cmbEAction.entry.set_text( iter.get_value(4) )
      @spnBtnENo.set_value( iter.get_value(5).to_i )
      @nbook.set_page( 5 )
    when "MEAS"
      @cmbMAction.entry.set_text( iter.get_value(4) )
      @spnBtnMTimes.set_value( iter.get_value(5).to_i )
      @nbook.set_page( 6 )
    end

    # 行番号
    change_line_no( iter.get_value(0).to_i )
  end

  # 行番号変更
  def change_line_no(no)
    @spnBtnLine.set_value( no )
  end

  # ステータスクリア
  def clear_status
    @lblStatus.each { |x| x.set_text( "" ) }
  end

  # コラムリストに追加/変更
  def clist_write( fmt )
    flag=nil
    @treeview.model.each do |model,path,iter|
      line = iter.get_value(0).to_i
      # 入れ替え
      if line == fmt[0].to_i
        treeview_add(fmt, iter)
        flag = true
        break
      # 挿入
      elsif line > fmt[0].to_i
        niter = @treeview.model.insert_before(iter)
        treeview_add(fmt, niter)
        flag = true
        break
      end
    end
    # 追加
    if flag == nil
      iter = @treeview.model.append
      treeview_add(fmt, iter)
    end
  end

  # 削除
  def delete_clicked
    cur_ary = []
    @treeview.selection.selected_each { |model, path, iter| cur_ary.push iter }
    cur_ary.each do |iter|
      @treeview.model.remove( iter )
    end

    # ステータスクリア
    clear_status

    @clist_change = true
  end

  # ReNo.
  def reno_clicked
    # ステータスクリア
    clear_status

    # 新行番号ハッシュの作成
    no = 10
    hash = {}
    @treeview.model.each do |model,path,iter|
      hash[iter.get_value(0).to_i] = no
      no += 10
    end
    # goto先の確認(コメント行も有効)
    @treeview.model.each do |model,path,iter|
      if (iter.get_value(4) == 'IF' || iter.get_value(4) == 'UNLESS' || iter.get_value(4) == 'GOTO') && hash[iter.get_value(5).to_i] == nil
        @lblStatus[0].set_text( "行番号#{iter.get_value(0).to_i}のgoto先が見つかりません" )
        return
      end
    end
    # 新行番号の割り振り
    @treeview.model.each do |model,path,iter|
      iter.set_value(0,"%4d" % [hash[iter.get_value(0).to_i]])

      # if / unless / goto対応
      case iter.get_value(4)
      when 'IF'
        iter.set_value(5,"#{hash[iter.get_value(5).to_i]}")
      when 'UNLESS'
        iter.set_value(5,"#{hash[iter.get_value(5).to_i]}")
      when 'GOTO'
        iter.set_value(5,"#{hash[iter.get_value(5).to_i]}")
      end
    end

    @clist_change = true
  end

  # PPM書込み
  def ppm_write_clicked
    fmt = [ "%4d" % [ @spnBtnLine.value_as_int ],
            "#{@edtPComment.text}",
            'PPM',
            "#{@spnBtnNode.value_as_int}",
            "#{@spnBtnPMNo.value_as_int}",
            "#{@cmbPAction.entry.text}",
            "#{@spnBtnPPulse.value_as_int}",
            "#{@spnBtnPStart.value_as_int}",
            "#{@spnBtnPMax.value_as_int}",
            "#{@spnBtnPUSlope.value_as_int}",
            "#{@spnBtnPDSlope.value_as_int}",
            "#{@spnBtnPRatio.value_as_int}",
            "#{@spnBtnR6.value_as_int}",
            "#{@spnBtnR7.value_as_int}",
            "#{@spnBtnR8.value_as_int}" ]
    clist_write( fmt )
    # ステータスクリア
    clear_status

    @clist_change = true
  end

  # DCM書込み
  def dcm_write_clicked
    fmt = [ "%4d" % [ @spnBtnLine.value_as_int ],
            "#{@edtAComment.text}", 
            'DCM',
            "#{@spnBtnANode.value_as_int}", 
            "#{@spnBtnAPort.value_as_int}",
            "#{@cmbAAction.entry.text}", 
            "#{@spnBtnAAdd.text}",
            "-", "-", "-", "-", "-", "-", "-", "-" ]
    clist_write( fmt )
    # ステータスクリア
    clear_status

    @clist_change = true
  end

  # DIO書込み
  def dio1_write_clicked
    fmt = [ "%4d" % [ @spnBtnLine.value_as_int ],
            "#{@edtDComment.text}", 
            'DIO',
            "#{@spnBtnDNode.value_as_int}", 
            "#{@spnBtnDPort.value_as_int}",
            "#{@cmbDAction.entry.text}", 
            "#{@edtDBitNo.text}",
            "-", "-", "-", "-", "-", "-", "-", "-" ]
    clist_write( fmt )
    # ステータスクリア
    clear_status

    @clist_change = true
  end

  # SIO書込み
  def sio_write_clicked
    fmt = [ "%4d" % [ @spnBtnLine.value_as_int ],
            "#{@edtSComment.text}", 
            'SIO',
            '',
            "#{@spnBtnSCh.value_as_int}",
            ( @cbCr.active? ) ? 'SIO ADD CR' : 'SIO NORMAL',
            "#{@edtSText.text}",
            "-", "-", "-", "-", "-", "-", "-", "-" ]
    clist_write( fmt )
    # ステータスクリア
    clear_status

    @clist_change = true
  end

  # WAIT書込み
  def wait_write_clicked
    fmt = [ "%4d" % [ @spnBtnLine.value_as_int ],
            "#{@edtWComment.text}", 
            'WAIT',
            '',
            '',
            'WAIT(msec)',
            "#{@spnBtnWTimes.value_as_int}",
            "-", "-", "-", "-", "-", "-", "-", "-" ]
    clist_write( fmt )
    # ステータスクリア
    clear_status

    @clist_change = true
  end

  # IF書込み
  def if_write_clicked
    fmt = [ "%4d" % [ @spnBtnLine.value_as_int ],
            "#{@edtIComment.text}", 
            'IF',
            '',
            '',
            'IF',
            "#{@spnBtnIGoto.value_as_int}",
            "#{@edtIMask.text}",
            "#{@edtIValue.text}",
            "-", "-", "-", "-", "-", "-" ]
    clist_write( fmt )
    # ステータスクリア
    clear_status

    @clist_change = true
  end

  # UNLESS書込み
  def unless_write_clicked
    fmt = [ "%4d" % [ @spnBtnLine.value_as_int ],
            "#{@edtUComment.text}", 
            'UNLESS',
            '',
            '',
            'UNLESS',
            "#{@spnBtnUGoto.value_as_int}",
            "#{@edtUMask.text}",
            "#{@edtUValue.text}",
            "-", "-", "-", "-", "-", "-" ]
    clist_write( fmt )
    # ステータスクリア
    clear_status

    @clist_change = true
  end

  # GOTO書込み
  def goto_write_clicked
    fmt = [ "%4d" % [ @spnBtnLine.value_as_int ],
            "#{@edtGComment.text}", 
            'GOTO',
            '',
            '',
            (@cbGotoEnd.active?) ? "GOTO_END" : "GOTO",
            "#{@spnBtnGoto.value_as_int}",
            "-", "-", "-", "-", "-", "-", "-", "-" ]
    clist_write( fmt )
    # ステータスクリア
    clear_status

    @clist_change = true
  end

  # ERROR書込み
  def error_write_clicked
    fmt = [ "%4d" % [ @spnBtnLine.value_as_int ],
            "#{@edtErComment.text}", 
            'ERROR',
            "#{@spnBtnErNode.value_as_int}", 
            "#{@spnBtnErPort.value_as_int}",
            'ERROR',
            "#{@edtErBitNo.text}",
            "-", "-", "-", "-", "-", "-", "-", "-" ]
    clist_write( fmt )
    # ステータスクリア
    clear_status

    @clist_change = true
  end

  # EVENT書込み
  def event_write_clicked
    fmt = [ "%4d" % [ @spnBtnLine.value_as_int ],
            "#{@edtEComment.text}", 
            'EVENT',
            '',
            '',
            "#{@cmbEAction.entry.text}", 
            "#{@spnBtnENo.value_as_int}",
            "-", "-", "-", "-", "-", "-", "-", "-" ]
    clist_write( fmt )
    # ステータスクリア
    clear_status

    @clist_change = true
  end

  # MEAS書込み
  def meas_write_clicked
    fmt = [ "%4d" % [ @spnBtnLine.value_as_int ],
            "#{@edtMComment.text}", 
            'MEAS',
            '',
            '',
            "#{@cmbMAction.entry.text}", 
            "#{@spnBtnMTimes.value_as_int}",
            "-", "-", "-", "-", "-", "-", "-", "-" ]
    clist_write( fmt )
    # ステータスクリア
    clear_status

    @clist_change = true
  end

  # Go
  def go_clicked
    # 実行範囲を取得
    cur_ary = []
    @treeview.selection.selected_each { |model, path, iter| cur_ary.push iter.get_value(0).to_i }

    if cur_ary[0]
      execute( cur_ary[0], cur_ary[-1], 0 )
      # main formへline通知
      $main_form.start[ @my_console_no-1 ].set_value( cur_ary[0] )
      $main_form.stop[ @my_console_no-1 ].set_value( cur_ary[-1] )
    else
      @lblStatus[0].set_text( "実行範囲が選ばれていません" )
      # 赤文字
      style = Gtk::Style.new
      style.font_desc = Pango::FontDescription.new("Monospace 14")
      style.set_fg(Gtk::STATE_NORMAL, 65535, 0, 0)
      @lblStatus[0].style = style
    end
  end

  # Step(次)
  def step_clicked
    cur_line_exec( 1 )
  end

  # More(もう一度)
  def more_clicked
    cur_line_exec( 0 )
  end

  # Step/More実行
  def cur_line_exec( mode )
    # 実行行を取得
    cur_ary = []
    @treeview.selection.selected_each { |model, path, iter| cur_ary.push iter.get_value(0).to_i }

    if cur_ary[0]
      execute( cur_ary[0], cur_ary[0], mode )
    else
      @lblStatus[0].set_text( "実行行が選ばれていません" )
      # 赤文字
      style = Gtk::Style.new
      style.font_desc = Pango::FontDescription.new("Monospace 14")
      style.set_fg(Gtk::STATE_NORMAL, 65535, 0, 0)
      @lblStatus[0].style = style
    end

    return cur_ary[0]
  end

  # PPM微調整
  def ppm_adjust_clicked
    # 実行行を取得
    cur_ary = []
    @treeview.selection.selected_each { |model, path, iter| cur_ary.push iter.get_value(0).to_i }

    ppm_adjust_pulse = nil

    # Actionファイルに書き込む
    save_to_action_file

    open_flag = nil
    # Actionファイルを読み込む
    fname = $main_form.file_action + "#{@my_console_no}" + Kakuchou_si
    if File.exist?( fname )
      open( fname, "r" ) do |f|
        while line = f.gets
          ary = (line.chop).split( /,/ )
          # 絶対パルス移動行であれば調整画面を開く
          if ary[0].to_i == cur_ary[0] && ary[2] == 'PPM' && $act_hash_ppm.key(ary[5]) == 0x15
            ppm_adjust_pulse = PPMAdjustPulse.new(@my_console_no, ary).pulse
            open_flag = true
            break
          end
        end
      end
    end

    unless open_flag
      @lblStatus[0].set_text( "#{$act_hash_ppm[0x15]} が選ばれていません" )
      # 赤文字
      style = Gtk::Style.new
      style.font_desc = Pango::FontDescription.new("Monospace 14")
      style.set_fg(Gtk::STATE_NORMAL, 65535, 0, 0)
      @lblStatus[0].style = style
    else
      @lblStatus[0].set_text( '' )
    end

    # 登録されたパルスを書き込む
    if ppm_adjust_pulse
      # TreeViewに表示する
      @treeview.model.each do |model,path,iter|
        if iter.get_value(0).to_i == cur_ary[0]
          iter.set_value(5, "#{ppm_adjust_pulse}")
        end
      end
      # Editにも表示する
      if @spnBtnLine.value_as_int == cur_ary[0]
        @spnBtnPPulse.set_value( ppm_adjust_pulse )
      end
      # Actionファイルに書き込む
      @clist_change = true
      save_to_action_file
    end
  end

  def execute ( start_line, stop_line, mode )
    # Actionが停止であることを確認する
    return if $main_form.status[ @my_console_no-1 ].text != "stop"

    # Actionファイルに書き込む
    save_to_action_file

    # 行番号ハッシュの作成
    hash = {}
    @treeview.model.each do |model,path,iter|
      hash[iter.get_value(0).to_i] = 1
    end
    # goto先の確認(コメント行は無視)
    @treeview.model.each do |model,path,iter|
      if iter.get_value(1)[0,1] != '#' && (iter.get_value(4) == 'IF' || iter.get_value(4) == 'UNLESS' || iter.get_value(4) == 'GOTO') && hash[iter.get_value(5).to_i] == nil
        @lblStatus[0].set_text( "行番号#{iter.get_value(0).to_i}のgoto先が見つかりません" )
        return
      end
    end

    # A/Dを開く
    if $ad_do
      $fd_ad.close if $fd_ad
      $fd_ad = nil

      if File.exist? ( ADDevFile )
        begin
          $fd_ad = open( ADDevFile, 'r' )
        rescue
        end
      end

      # A/Dが開けなければ終了
      if $fd_ad == nil
        @lblStatus[0].set_text "#{ADDevFile}のOPENに失敗しました!!"
        return
      else
        @lblStatus[0].set_text ''
      end
      $ad_log.close if $ad_log
      $ad_log = open( "#{ENV['HOME']}/public/" + Time.now.strftime( "A%y%m%d-%H%M%S.csv" ), 'w' )
    end

=begin
    # 60行以上は動作できません
    line_count = 0
    fname = $main_form.file_action + "#{@my_console_no}" + Kakuchou_si
    if File.exist?( fname )
      open( fname, "r" ) do |f|
        while line = f.gets
          ary = (line.chop).split( /,/ )

          next if ary[0].to_i < start_line
          next if ary[0].to_i > stop_line
          next if (ary[1])[ 0, 1 ] == "#"

          line_count += 1
        end
      end
    end
    if line_count > 60
      @lblStatus[0].set_text '60行以上の動作は実行できません!!'
      return
    end
=end

    if $sock_port.open_err == nil
      # ActionプロセスへREADY要求
      $sock_port.nt_send( [STX, 0x07, 0x00, 0x00, 0xC014, @my_console_no, 0x00, ETX], 'C4nC3' )

      # Actionファイルを読み込む
      fname = $main_form.file_action + "#{@my_console_no}" + Kakuchou_si
      if File.exist?( fname )
        open( fname, "r" ) do |f|
          while line = f.gets
            ary = (line.chop).split( /,/ )

            next if ary[0].to_i < start_line
            next if ary[0].to_i > stop_line
            next if (ary[1])[ 0, 1 ] == "#"

            action_info_send(@my_console_no, ary)
          end
        end
      end
      # ActionプロセスへSTART要求
      $main_form.status[ @my_console_no-1 ].set_text "run"
      $sock_port.nt_send( [STX, 0x11, 0x00, mode, 0xC015, @my_console_no, @spnBtnTimes.value_as_int, 0, 0, 0, 0, ETX], 'C4nCNn3C2' )
    else
      @lblStatus[0].set_text( "Socket送受信エラー!!" )
      # 赤文字
      style = Gtk::Style.new
      style.font_desc = Pango::FontDescription.new("Monospace 14")
      style.set_fg(Gtk::STATE_NORMAL, 65535, 0, 0)
      @lblStatus[0].style = style
    end
  end

  # Stop
  def stop_clicked
    $sock_port.nt_send( [STX, 0x07, 0x00, 0x00, 0xC016, @my_console_no, 0x00, ETX], 'C4nC3' ) if $sock_port.open_err == nil
  end

  # Actionファイルに書き込む
  def save_to_action_file

    # main formへTitle通知
    $main_form.name[ @my_console_no-1 ].set_text( @edtTitle.text )

    return if !@clist_change

    fname = $main_form.file_action + "#{@my_console_no}" + Kakuchou_si
    open( fname, "w" ) do |f|
      @treeview.model.each do |model,path,iter|
        ( 0..13 ).each do |j|
          # 行番号
          if j == 0
            f << "%4d" % [ iter.get_value(j) ]
          # ノードとモータ番号
          elsif j == 3
            if /-/ =~ iter.get_value(j) # ノードとモータ番号
              f << ",#{iter.get_value(j)}".gsub(/-/, ',')
            else
              f << ",,#{iter.get_value(j)}"
            end
          else
            f << ",#{iter.get_value(j)}"
          end
        end
        f << "\n"
      end
    end

    @clist_change = nil
  end

  # ACTION treeviewに追加
  def treeview_add(line, iter)
    node = ''
    line.each_with_index do |dt,i|
      if i == 0 || i == 1 || i == 2
        iter.set_value(i, dt)
      elsif i == 3
        node = dt
      elsif i == 4
        iter.set_value(3, (node == '' ) ? dt : node + '-' + dt)
      else
        iter.set_value(i-1, dt)
      end
    end
  end
end

# Main画面
class Gtn

  attr_accessor :prj_no, :console_opened, :enPrj, :name, :start, :stop, :status, :main_sts, :file_tnet, :file_ppm, :file_dcm, :file_config, :file_action

  def initialize( size )
#    @prj_no = 1
#    @file_tnet   = "#{Prjs[@prj_no]}/tnet#{Kakuchou_si}"
#    @file_ppm    = "#{Prjs[@prj_no]}/ptable#{Kakuchou_si}"
#    @file_config = "#{Prjs[@prj_no]}/config#{Kakuchou_si}"

    @size = size
    @console_opened = Array.new( size+1, nil )
    @name       = []
    @c_open     = []
    @start      = []
    @stop       = []
    @loop       = []
    @delay_no   = []
    @delay_time = []
    @delay_sec  = []
    @run        = []
    @status     = []

    # main Windowの作成
    form = Gtk::Window.new()
    form.set_title( 'gtn コンソール Ver.' + VER );
    form.signal_connect( 'delete_event' ){ exit_seq }
    form.window_position = Gtk::Window::POS_CENTER
    # 上部メニューの作成
    @enPrj    = Gtk::Entry.new()
    @enPrj.set_editable( false )
    btnSelPrj = Gtk::Button.new( '選択' )
#   btnTnet   = Gtk::Button.new( 'TNet接続' )
    btnDInfo  = Gtk::Button.new( 'DCMモータ情報' )
    btnMInfo  = Gtk::Button.new( 'PPMモータ情報' )
    btnMkPrj  = Gtk::Button.new( 'プロジェクト作成' )
    btnCopy   = Gtk::Button.new( 'コピー' )
    btnDelete = Gtk::Button.new( '削除' )
#   btnSio    = Gtk::Button.new( 'SIO設定' )
#   btnADDel  = Gtk::Button.new( 'A/Dファイル削除' )
    group1 = Gtk::Table.new( 1, 60, false )
    group1.attach( @enPrj,                   0, 46, 0, 1 )
    group1.attach( btnSelPrj,               46, 48, 0, 1 )
    group1.attach( btnDInfo,                48, 50, 0, 1 )
    group1.attach( btnMInfo,                50, 52, 0, 1 )
    group1.attach( Gtk::Label.new(' '),     52, 53, 0, 1 )
    group1.attach( btnMkPrj,                53, 55, 0, 1 )
    group1.attach( btnCopy,                 55, 57, 0, 1 )
    group1.attach( btnDelete,               57, 59, 0, 1 )

    # 中段作成
    group2 = Gtk::Table.new( 2, 2, false )
    group2.attach( Gtk::Label.new( '名前' ),        0, 11, 0, 1 )
    group2.attach( Gtk::Label.new( 'START' ),      12, 13, 0, 1 )
    group2.attach( Gtk::Label.new( '〜' ),         13, 14, 0, 1 )
    group2.attach( Gtk::Label.new( 'STOP' ),       14, 15, 0, 1 )
    group2.attach( Gtk::Label.new( '繰返し' ),     15, 16, 0, 1 )
    group2.attach( Gtk::Label.new( '     ' ),      16, 17, 0, 1 )
    group2.attach( Gtk::Label.new( '遅延番号' ),   17, 18, 0, 1 )
    group2.attach( Gtk::Label.new( '遅延数' ),     18, 19, 0, 1 )
    group2.attach( Gtk::Label.new( '遅延時間S' ),  19, 20, 0, 1 )
    group2.attach( Gtk::Label.new( '   ' ),        20, 21, 0, 1 )

    # 動作プロセス生成
    ( 1..@size ).each do |i|
      @name[ i-1 ]       = Gtk::Label.new( "" )
      @c_open[ i-1 ]     = Gtk::Button.new( "開く:#{i}" )
      tmp                = Gtk::Adjustment.new( 1, 1, 9999, 1, 2, 0 )
      @start[ i-1 ]      = Gtk::SpinButton.new( tmp, 0, 0 )
      tmp                = Gtk::Adjustment.new( 1, 1, 9999, 1, 2, 0 )
      @stop[ i-1 ]       = Gtk::SpinButton.new( tmp, 0, 0 )
      tmp                = Gtk::Adjustment.new( 1, 0, 999999, 1, 2, 0 )
      @loop[ i-1 ]       = Gtk::SpinButton.new( tmp, 0, 0 )
      tmp                = Gtk::Adjustment.new( 0, 0, 100, 1, 2, 0 )
      @delay_no[ i-1 ]   = Gtk::SpinButton.new( tmp, 0, 0 )
      tmp                = Gtk::Adjustment.new( 0, 0, 9999, 1, 2, 0 )
      @delay_time[ i-1 ] = Gtk::SpinButton.new( tmp, 0, 0 )
      tmp                = Gtk::Adjustment.new( 0, 0, 9999, 1, 2, 0 )
      @delay_sec[ i-1 ]  = Gtk::SpinButton.new( tmp, 0, 0 )
      @run[ i-1 ]        = Gtk::CheckButton.new( "run" )
      @status[ i-1 ]     = Gtk::Label.new( "stop" )
      # シグナル定義
      @c_open[ i-1 ].signal_connect( 'clicked' ){ open_action(i) }
      # attach
      group2.attach( @name[ i-1 ],          0, 11, i, i+1 )
      group2.attach( @c_open[ i-1 ],       11, 12, i, i+1 )
      group2.attach( @start[ i-1 ],        12, 13, i, i+1 )
      group2.attach( @stop[ i-1 ],         14, 15, i, i+1 )
      group2.attach( @loop[ i-1 ],         15, 16, i, i+1 )
      group2.attach( @delay_no[ i-1 ],     17, 18, i, i+1 )
      group2.attach( @delay_time[ i-1 ],   18, 19, i, i+1 )
      group2.attach( @delay_sec[ i-1 ],    19, 20, i, i+1 )
      group2.attach( @run[ i-1 ],          21, 22, i, i+1 )
      group2.attach( @status[ i-1 ],       22, 23, i, i+1 )
    end

    # 下部メニューの作成
    btnStart    = Gtk::Button.new( 'START' )
    btnStop     = Gtk::Button.new( 'EMG STOP' )
    btnClose    = Gtk::Button.new( '終了' )
    @main_sts = Gtk::Label.new( "" )

    group3 = Gtk::Table.new( 2, 2, false )
    group3.attach( @main_sts,                0,  3, 0, 1 )
    group3.attach( btnStart,                 0,  1, 1, 2 )
    group3.attach( btnStop,                  1,  2, 1, 2 )
    group3.attach( btnClose,                 2,  3, 1, 2 )

    # 全体
    table = Gtk::Table.new( 2, 62, false )
    table.attach( Gtk::Label.new( "  " ),   0,  1, 0, 1 )
    table.attach( group1,                   1, 60, 0, 1 )
    table.attach( Gtk::Label.new( "  " ),  60, 61, 0, 1 )
    table.attach( group2,                   1, 60, 1, 2 )
    table.attach( group3,                   1, 60, 2, 3 )

    form.add table
    form.show_all

    # シグナル定義
    btnClose.signal_connect( 'clicked' ){ exit_seq }
    btnSelPrj.signal_connect( 'clicked' ){ sel_prj_clicked }
    btnMkPrj.signal_connect( 'clicked' ){ MkProject.new }
    btnDInfo.signal_connect( 'clicked' ){ dcm_clicked }
    btnMInfo.signal_connect( 'clicked' ){ moter_clicked }
    btnCopy.signal_connect( 'clicked' ){ copy_clicked }
    btnDelete.signal_connect( 'clicked' ){ delete_clicked }
#   btnSio.signal_connect( 'clicked' ){ SioEdit.new }
#   btnADDel.signal_connect( 'clicked' ){ system( "/bin/rm -f #{ENV['HOME']}/public/*" ) }
    btnStart.signal_connect( 'clicked' ){ start_clicked }
    btnStop.signal_connect( 'clicked' ){ stop_clicked }
  end


  # 終了処理
  def exit_seq
    # Actionモジュール停止
#   $sock_port.nt_send( [STX, 0x06, 0x00, 0x00, 0xC013, 0x00, ETX], 'C4nC2' ) if $sock_port.open_err == nil

    # configファイルへ書き込む
    save_to_config_file

    Gtk::main_quit
  end

  # プロジェクト選択ボタンクリック
  def sel_prj_clicked

    return  if !File.exist?( PrjNames )

    # 全Actionが停止であることを確認する
    return unless $main_form.status.map { |x| x.text }.uniq == ['stop']

    # 全コンソールが閉じていることを確認する
    @console_opened.each { |x| return if x }

    # configファイルに書き出す
    save_to_config_file

    SelProject.new

    # configファイルに書き出す
    save_to_config_file
  end

  # Tnet接続ボタンクリック
  def tnet_clicked
    # プロジェクトが選ばれている事
    return if $main_form.file_tnet == nil

    # 全Actionが停止であることを確認する
    return unless $main_form.status.map { |x| x.text }.uniq == ['stop']

    Tnet.new
  end

  # DCモータ情報ボタンクリック
  def dcm_clicked
    # プロジェクトが選ばれている事
    return if $main_form.file_dcm == nil

    # 全Actionが停止であることを確認する
    return unless $main_form.status.map { |x| x.text }.uniq == ['stop']

    DcmConf.new
  end

  # モータ情報ボタンクリック
  def moter_clicked
    # プロジェクトが選ばれている事
    return if $main_form.file_ppm == nil

    # 全Actionが停止であることを確認する
    return unless $main_form.status.map { |x| x.text }.uniq == ['stop']

    Moter.new
  end

  # コピーボタンクリック
  def copy_clicked
    # プロジェクトが選ばれている事
    return if $main_form.file_config == nil

    # 全Actionが停止であることを確認する
    return unless $main_form.status.map { |x| x.text }.uniq == ['stop']

    ActCopy.new( $main_form.prj_no )
  end

  # 削除ボタンクリック
  def delete_clicked
    # プロジェクトが選ばれている事
    return if $main_form.file_config == nil

    # 全Actionが停止であることを確認する
    return unless $main_form.status.map { |x| x.text }.uniq == ['stop']

    ActDelete.new( $main_form.prj_no )
  end

  # 開く
  def open_action( no )
    # プロジェクトが選ばれている事
    return if $main_form.file_config == nil
    return if @console_opened[ no ]

    @console_opened[ no ] = Input.new( no )
  end

  # Start
  def start_clicked
    # プロジェクトが選ばれている事
    return if $main_form.file_config == nil

    # 全Actionが停止であることを確認する
    return unless $main_form.status.map { |x| x.text }.uniq == ['stop']

    ready = []
    ( 1..@size ).each do |i|

      next if !@run[ i-1 ].active?
      if $sock_port.open_err == nil
        # ActionプロセスへREADY要求
        $sock_port.nt_send( [STX, 0x07, 0x00, 0x00, 0xC014, i, 0x00, ETX], 'C4nC3' )
        ready.push i

        # Actionファイルを読み込む
        fname = $main_form.file_action + "#{i}" + Kakuchou_si
        if File.exist?( fname )
          open( fname, "r" ) do |f|
            while line = f.gets
              ary = (line.chop).split( /,/ )

              next if ary[0].to_i < @start[ i-1 ].value_as_int
              next if ary[0].to_i > @stop[ i-1 ].value_as_int
              next if (ary[1])[ 0, 1 ] == "#"

              action_info_send(i, ary)
            end
          end
        end
        $main_form.main_sts.set_text( "RUN" )
      else
        $main_form.main_sts.set_text( "Socket送受信エラー!!" )
      end
    end

    # ActionプロセスへSTART要求
    ready.each do |i|
      $sock_port.nt_send( [STX, 0x11, 0x00, 0x00, 0xC015, i, @loop[ i-1 ].value_as_int,
                                                              @delay_no[ i-1 ].value_as_int,
                                                              @delay_time[ i-1 ].value_as_int,
                                                              @delay_sec[ i-1 ].value_as_int, 0, ETX], 'C4nCNn3C2' )
      $main_form.status[ i-1 ].set_text "run"
    end
  end

  # EMG
  def stop_clicked
    # プロジェクトが選ばれている事
    return if $main_form.file_config == nil

    $sock_port.nt_send( [STX, 0x07, 0x00, 0x00, 0xC016, 0, 0x00, ETX], 'C4nC3' ) if $sock_port.open_err == nil

    # モータレジスタの初期化
    motor_reg_init
    dcmotor_reg_init
  end

  # configファイルに書き出す
  def save_to_config_file
    # プロジェクトが選ばれている事
    return if $main_form.file_config == nil

    open( $main_form.file_config, 'w' ) do |f|
      ( 1..@size ).each do |i|
        f << "#{i},"
        f << "#{@name[ i-1 ].text},"
        f << "#{@start[ i-1 ].value_as_int},"
        f << "#{@stop[ i-1 ].value_as_int},"
        f << "#{@loop[ i-1 ].value_as_int},"
        f << "#{@delay_no[ i-1 ].value_as_int},"
        f << "#{@delay_time[ i-1 ].value_as_int},"
        f << "#{@delay_sec[ i-1 ].value_as_int}"
        f << "\n"
      end
    end
  end

  # 表示
  def show
    if File.exist?( $main_form.file_config )
      open( $main_form.file_config, 'r' ) do |f|
        while line = f.gets
          ary = line.chop.split( /,/ )
          i   = ary[0].to_i
          if i <= @size && ary.size == 8
            @name[ i-1 ].set_text( ary[1] )
            @start[ i-1 ].set_value( ary[2].to_i )
            @stop[ i-1 ].set_value( ary[3].to_i )
            @loop[ i-1 ].set_value( ary[4].to_i )
            @delay_no[ i-1 ].set_value( ary[5].to_i )
            @delay_time[ i-1 ].set_value( ary[6].to_i )
            @delay_sec[ i-1 ].set_value( ary[7].to_i )
          end
        end
      end
    else
      ( 1..@size ).each do |i|
        @name[ i-1 ].set_text( '' )
        @start[ i-1 ].set_value( 1 )
        @stop[ i-1 ].set_value( 1 )
        @loop[ i-1 ].set_value( 1 )
        @delay_no[ i-1 ].set_value( 0 )
        @delay_time[ i-1 ].set_value( 0 )
        @delay_sec[ i-1 ].set_value( 0 )
      end
    end
  end
end


#-------------------------------------------------------------------------------
# ここからSTART
#-------------------------------------------------------------------------------

# パラメータディレクトリを生成する
Prjs.each { |x|  Dir.mkdir( x ) if File.exist?( x ) == false }

$sio_do = false
$ad_do  = false

# 引数を分解
while arg = ARGV.shift
  case arg
  when "-sio"
    $sio_do = true
  when "-ad"
    $ad_do = true
  end
end

# 通信オブジェクト生成
$sock_port = MySocket.new

# MAIN画面表示
$main_form = Gtn.new( 20 )

if File.exist?( PrjNames )
  SelProject.new 
else
  MkProject.new
end

# RTタスクからの受信処理
Gtk.timeout_add( 200 ) do
  $sock_port.nt_recv_each do |rcv_msg|
    stx, len, type, dmy, id, my_no, dsp, line, msg, crc, etx = rcv_msg.pack('C*').unpack('C4nC3A32C2')

    # ベース画面の各状態をstopに
    if my_no > 0 && line == 1 && ( msg == 'success!!' || msg[0,3] == 'ERR' || msg[0,4] == 'STOP' )
      $main_form.status[ my_no-1 ].set_text( 'stop' )
      # 全てstopであれば全体ステータスを表示
      if $main_form.status.map { |x| x.text }.uniq == ['stop']
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
  end
  true
end

=begin
Gtk.timeout_add( 500 ) do
  # SIO受信
  if $sio_form
    ary = []
    $sio_dev.each do |tty|
      next if tty == nil || tty == ''
      ary.push open( tty, 'r' )
    end

    ret = select( ary, [], [], 0.01 )
#p ret
    if ret && ret[0]
      ret[0].each do |f|
        $sio_form.text.insert_text( "R:#{f.gets}¥n", 0 )
      end
    end
    ary.each { |tty| tty.close }

  # SIO受信表示画面が閉じられているので再表示
  else
    $sio_form = SioWindow.new if $sio_do
  end

  # A/D受信
  if $fd_ad
    n = $fd_ad.ioctl( 3 )                 # 受信数取得

    # 16チャンネル毎に読み出す
    while n>31
      rd = $fd_ad.sysread(32).unpack("S*")    # read
      n -= 32

      buf = []
      rd.each { |p| buf.push( "%1.1f" % [(p.to_f-32768.0) * 10000.0 / 65536.0 ] ) }
      $ad_log << buf.join(',') + "¥n"
    end

    # A/D終了
    (1..20).each do |i|
      break if $main_form.status[ i-1 ].text != "stop"
      if i==20 && n < 1
        $ad_log.close
        $ad_log = nil
        $fd_ad.close
        $fd_ad = nil
      end
    end
  end

  true
end
=end

Gtk.main

