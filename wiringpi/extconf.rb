require "mkmf"
if have_library('wiringPi')
  create_makefile("WiringPiSpi")
end

