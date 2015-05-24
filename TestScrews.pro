
# -----------------------------------------------------------------------------
# Screws project profile
#
# \file
# \author  Christos Bergeles
# \date    2014-10-21
# -----------------------------------------------------------------------------

TEMPLATE   = app

TARGET     = TestScrews

DESTDIR    = ../../../bin
DLLDESTDIR = ../../../bin

# Set high warn level (warn 4 on MSVC)
WARN = HIGH

# Add used projects here (see included pri files below for available projects)
CONFIG += dll ML MLBase Screws

include(../../../Configuration/CustomLibraries_General.pri)

MLAB_PACKAGES += \
     MeVisLab_Standard

# make sure that this file is included after CONFIG and MLAB_PACKAGES
include ($(MLAB_MeVis_Foundation)/Configuration/IncludePackages.pri)

# Enable ML deprecated API warnings. To completely disable the deprecated API, change WARN to DISABLE.
DEFINES += ML_DISABLE_DEPRECATED

HEADERS += \
  src/screws.hpp

SOURCES += \
  src/testScrews.cpp
