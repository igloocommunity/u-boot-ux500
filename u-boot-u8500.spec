#Defines
%define base_name u-boot
%define variant_name u8500
%define base_version 2009.11
%define boot_path /boot

Name: %{base_name}-%{variant_name}
Release: 2
Version: %{base_version}
License: GPL
URL: http://www.denx.de/wiki/U-Boot
Source0: %{base_name}-%{base_version}.tar.bz2
Source1: localversion-meegobuild
Source100: %{base_name}-rpmlintrc

Summary: Das U-Boot boot loader binary
Group: Binary

%description
This package contains the source code for Das U-Boot.

U-Boot is a boot loader for Embedded boards based on PowerPC, ARM,
MIPS and several other processors, which can be installed in a boot
ROM and used to initialize and test the hardware or to download and
run application code.

%package -n %{base_name}-tools
Summary: Das U-Boot boot image maker
Group: Development/Tools

%description -n %{base_name}-tools
U-Boot utility for creating bootable kernel images.

%prep
%setup -q -n %{base_name}-%{base_version}
cp %{SOURCE1} .

%build
#Make default config for variant
make %{variant_name}_secboot_config

#Build-id needed/wanted by rpmbuild
export LDFLAGS="$LDFLAGS --build-id"

make u-boot

#Additional tools
make -C tools/mk_envimg

%install
#Build-id needed by rpm
export LDFLAGS="$LDFLAGS --build-id"

#Binaries
mkdir -p %{buildroot}%{_bindir} %{buildroot}%{boot_path}
install -m 755 %{_builddir}/u-boot-%{base_version}/u-boot %{buildroot}%{boot_path}
install -m 644 %{_builddir}/u-boot-%{base_version}/u-boot.map %{buildroot}%{boot_path}
install -m 644 %{_builddir}/u-boot-%{base_version}/u-boot.lds %{buildroot}%{boot_path}
install -m 755 %{_builddir}/u-boot-%{base_version}/tools/mkimage %{buildroot}%{_bindir}
install -m 755 %{_builddir}/u-boot-%{base_version}/tools/img2srec %{buildroot}%{_bindir}
install -m 755 %{_builddir}/u-boot-%{base_version}/tools/mk_envimg/mk_envimg %{buildroot}%{_bindir}
install -m 644 %{_builddir}/u-boot-%{base_version}/tools/logos/ste-rgb565.bin %{buildroot}%{boot_path}/splash.bin

#Remove unwanted sections
objcopy --gap-fill=0xff --remove-section=.note.gnu.build-id -O binary %{buildroot}%{boot_path}/u-boot %{buildroot}%{boot_path}/u-boot.bin

%clean
rm -rf %{buildroot}/*

%files
%defattr(-,root,root)
%{boot_path}/u-boot.bin
%{boot_path}/u-boot.map
%{boot_path}/u-boot
%{boot_path}/u-boot.lds
%{boot_path}/splash.bin

%files -n %{base_name}-tools
%defattr(-,root,root)
%{_bindir}/mkimage
%{_bindir}/img2srec
%{_bindir}/mk_envimg
