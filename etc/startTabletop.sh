# RCIS
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcis /home/robocomp/robocomp/components/perception/etc/genericPointCloud.xml -p 20202'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rcis'
sleep 3

# Ice Storm
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/perception/etc'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'icebox --Ice.Config=config.icebox'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'storm'
sleep 1

#primeSenseComp
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/experimental/primeSenseComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/primeSenseComp  --Ice.Config=/home/robocomp/robocomp/components/perception/etc/primesense.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'primeSenseComp'

# AprilTags
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/apriltagscomp --Ice.Config=/home/robocomp/robocomp/components/perception/etc/aprilcomp_rcis.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'april Comp'
sleep 1

# ObjectDetection
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/perception/components/objectDetection/build/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/objectdetection --Ice.Config=/home/robocomp/robocomp/components/perception/etc/objectDetection.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'ObjectDetection'
sleep 1

# AGM Executive
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/AGM/tools/AGMExecutive_robocomp'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'python AGMExecutive_robocomp.py --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/etc/executive_rcis.conf'
#qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'executive'
#sleep 1


# AprilTags
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robolab/components/apriltagsComp'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/apriltagscomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/etc/aprilcomp_rcis.conf'
#qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'april Comp'
#sleep 1


# Mission
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/missionAgent'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/missionagent --Ice.Config=config'
#qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'mission'



# ikComp
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematicsComp'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/lokiarmcomp  --Ice.Config=../../etc/ikComp_rcis.conf'
#qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'ikComp'


# ikComp
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematicsAgent'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/inversekinematicsagentcomp  --Ice.Config=../../etc/ikAgent_rcis.conf'
#qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'ikAgent'



# April Tags
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/perception/components/objectDetectorAgentComp'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/objectdetectoragentcomp  --Ice.Config=../../etc/april_rcis.conf'
#qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'object detector agent'


# Move tag
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/misc'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcmonitor moveTag.rcm'
#qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'moveTag'

# RCIS Cognitive
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcis /home/robocomp/robocomp/components/robocomp-ursus/etc/ursusCog.xml -p 20202'
#qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rcis COG'
#sleep 3
 
# Model renderer
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/modelRenderer'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/modelrenderercomp  --Ice.Config=../../etc/modelrenderer.conf'
#qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'modelRenderer'
 
 
 
