######################################################################
#  Project Makefile
######################################################################

.PSEUDO: all node nodeCAN clean clobber nodeflash nodeovercan 

#main:
#	$(MAKE) -f Makefile.main -$(MAKEFLAGS)

#main:
#	$(MAKE) -f Makefile.node -$(MAKEFLAGS)
	
node:
	$(MAKE) -f Makefile.node -$(MAKEFLAGS)

nodeCAN:
	$(MAKE) -f Makefile.nodeCAN -$(MAKEFLAGS)
	
all:
	$(MAKE) -f Makefile.node -$(MAKEFLAGS)
	$(MAKE) -f Makefile.nodeCAN -$(MAKEFLAGS)

clean:
	$(MAKE) -f Makefile.node -$(MAKEFLAGS) clean
	$(MAKE) -f Makefile.nodeCAN -$(MAKEFLAGS) clean

clobber:
	$(MAKE) -f Makefile.node -$(MAKEFLAGS) clobber
	$(MAKE) -f Makefile.nodeCAN -$(MAKEFLAGS) clobber

#flash:
#	$(MAKE) -f Makefile.main -$(MAKEFLAGS) flash

nodeflash:
	$(MAKE) -f Makefile.node -$(MAKEFLAGS) clean_hex
	$(MAKE) -f Makefile.node -$(MAKEFLAGS) flash

nodeovercan:
	$(MAKE) -f Makefile.node -$(MAKEFLAGS) clean_hex
	$(MAKE) -f Makefile.nodeCAN -$(MAKEFLAGS) nocan

