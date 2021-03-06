
# -----------------------------------------------------------------------------
# Screws project profile
#
# \file
# \author  Christos Bergeles
# \date    2014-10-21
# -----------------------------------------------------------------------------

TEMPLATE   = lib

TARGET     = Screws

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
DEFINES += ML_DISABLE_DEPRECATED

HEADERS += \
  src/screws.hpp \
  src/translation.hpp \
  src/rotation.hpp \
  src/homogeneousTransform.hpp \
  src/skew.hpp \
  src/twist.hpp \
  src/adjoint.hpp \
  src/screwException.hpp \
  src/screwsInitLibrary.hpp \
  src/vector6.hpp

SOURCES += \
  src/screwException.cpp
