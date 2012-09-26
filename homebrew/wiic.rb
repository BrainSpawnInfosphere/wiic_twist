require 'formula'

class Wiic < Formula
  #url 'http://sourceforge.net/projects/wiic/files/latest/download?source=files'
  url 'http://sourceforge.net/projects/wiic/files/latest/download'
  #url 'https://wiic.svn.sourceforge.net/svnroot/wiic', :using => :svn
  #md5 '63fd633a6306ae9b334131b250a2f893'
  #url 'http://sourceforge.net/projects/wiic/files/WiiC/1.1/'
  version '1.1'
  
  homepage 'http://wiic.sourceforge.net'
  
  depends_on 'cmake' => :build
  #depends_on 'opencv'
  
  def patches
    DATA
  end

  def install

    system "mkdir src/build"
    
    Dir.chdir 'src/build' do
      system "cmake .."
      system "make"
      
      system "mkdir lib"
      system "cp wiic/*.dylib lib"
      system "cp wiicpp/*.dylib lib"
      system "cp ml/*.dylib lib"
      
      system "mkdir bbin"
      system "cp bin/wiic-logger bbin"
      system "cp bin/wiic-example bbin"
      system "cp bin/wiic-ml bbin"
      #system "cp bin/wiic-robot bbin"
      system "cp bin/wiicpp-example bbin"
      
      bin.install Dir['bbin/*']
      lib.install Dir['lib/*']
    end
    
    Dir.chdir 'src' do
      system "mkdir in"
      system "cp wiic/*.h in"
      system "cp wiicpp/*.h in"
      system "cp log/*.h in"
      #system "cp ml/*.h in"
      #system "rm -fr wiic"
      #system "mv in wiic"
      #include.install Dir['wiic']
      include.install Dir['in/*']
    end
  end
end

__END__
diff --svn a/src/wiic/io_mac.m b/src/wiic/io_mac.m 
Index: src/wiic/io_mac.m
--- a/src/wiic/io_mac.m
+++ b/src/wiic/io_mac.m	
@@ -585,7 +585,7 @@
 	for (; i < wiimotes; ++i) {
 		if(!(wm[i])) {
 			WIIC_ERROR("Trying to connect more Wiimotes than initialized");
-			return;
+			return 0;
 		}
 		
 		if (!WIIMOTE_IS_SET(wm[i], WIIMOTE_STATE_DEV_FOUND))
