
# -----------------------------------------------------------------------------
# Screws project profile
#
# \file
# \author  Christos Bergeles
# \date    2014-10-21
# -----------------------------------------------------------------------------

TEMPLATE   = app

TARGET     = TestScrews

DESTDIR    = ../../../lib
DLLDESTDIR = ../../../lib

# Set high warn level (warn 4 on MSVC)
WARN = HIGH

# Add used projects here (see included pri files below for available projects)
CONFIG += dll ML MLBase Eigen324

include(../../../Configuration/CustomLibraries_General.pri)

MLAB_PACKAGES += \
     MeVisLab_Standard

# make sure that this file is included after CONFIG and MLAB_PACKAGES
include ($(MLAB_MeVis_Foundation)/Configuration/IncludePackages.pri)

DEFINES += SCREWS_EXPORTS

# Enable ML deprecated API warnings. To completely disable the deprecated API, change WARN to DISABLE.
DEFINES += ML_WARN_DEPRECATED

HEADERS += \
  screws.hpp \
  translation.hpp \
  rotation.hpp \
  homogeneousTransform.hpp \
  skew.hpp \
  twist.hpp \
  adjoint.hpp \
  screwException.hpp \
  screwsInitLibrary.hpp \
    vector6.hpp

SOURCES += \
  screwException.cpp \
  testScrews.cpp \
#  translation.cpp \
#  rotation.cpp \
#  skew.cpp \
#  twist.cpp \
#  adjoint.cpp

