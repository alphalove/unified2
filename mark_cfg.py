import subprocess
from time import sleep
from collections import namedtuple

# infomation to generate _nocanc.conf
nocanEventServer = '127.0.0.1:4242'
nocanAuthToken = 'mark'
mqttServer = "127.0.0.1"
mqttTopicPrefix = "mt/"         #  prefix added to nocan channels when talking to mqtt server
mqttPubPrefix = "stat/"         #  prefix added by node for status commands (published to mqtt server)
mqttSubPrefix = "cmd/"          #  prefix added by node for control commands (subscribed to mqtt server)

# file path / name for new ihex fw file for upload
fileName = 'node.hex'

bootDelay = 12
krbRenameNode = '00'
krbClearLoadSubs = '12'         #  kroby dev command to clear all load controller subscriptions
krbSetLoadSubNext = '13'        #  kroby dev command to configure locd controller sub name and led mask

DevName = namedtuple('DevName', 'deviceID, friendlyName')

nodeNames2 = (  DevName("dev/B348", "dc/up/bed1"),
                DevName("dev/8A33", "dc/up/bed2"),           
             )

# Naming of nodes, will be prefixed with dev/ by node
nodeNames = {"dev/B348" : "dc/up/bed1", 
              "dev/8A33" : "dc/up/bed2",
              "dev/6AF2" : "dc/up/bathroom",
              "dev/3AD5" : "dc/up/bed3",
              "dev/3FD8" : "dc/up/hallway",
              "dev/0474" : "dc/up/stairs1",
              "dev/203A" : "dc/up/stairs2",
              "dev/F727" : "dc/up/lounge1",
              "dev/9E3E" : "dc/up/lounge2",
              "dev/F8A0" : "dc/up/lounge3",
              "dev/FB9B" : "dc/up/lounge4",
              "dev/0A02" : "dc/dn/garage1",
              "dev/5D9A" : "dc/dn/garage2",
              "dev/D763" : "dc/dn/pantry",
              "dev/91FF" : "dc/dn/kitchen",
              "dev/97EE" : "dc/dn/bifolds",
              "dev/0918" : "dc/dn/dining",
              "dev/C58B" : "dc/dn/dining-central",
              "dev/5988" : "dc/dn/lounge-central",
              "dev/1BFD" : "dc/dn/lounge",
              "dev/F7AA" : "dc/dn/entry1",
              "dev/2740" : "dc/dn/entry2"
}

# TODO, this could probably just be a list...
lightingZones = {'0' : ("light/dn/garage-all", "dev/dc/dn/garage1", 'E'),
                 '1' : ("light/dn/garage-all", "dev/dc/dn/garage2", 'F'),
                 '2' : ("light/dn/garage-no-roller", "dev/dc/dn/garage1", 'C'),
                 '3' : ("light/dn/garage-no-roller", "dev/dc/dn/garage2", '5'),
                 '4' : ("light/dn/pantry", "dev/dc/dn/pantry", '6'),
                 '5' : ("light/dn/kitchen", "dev/dc/dn/pantry", '8'),
                 '6' : ("light/dn/kitchen", "dev/dc/dn/kitchen", 'F'),
                 '7' : ("light/dn/kitchen-bench", "dev/dc/dn/kitchen", 'D'),
                 '8' : ("light/dn/dining", "dev/dc/dn/bifolds", 'C'),
                 '9' : ("light/dn/dining", "dev/dc/dn/dining", '6'),
                '10' : ("light/dn/dining", "dev/dc/dn/dining-central", 'F'),
                '11' : ("light/dn/lounge", "dev/dc/dn/lounge-central", '6'),
                '12' : ("light/dn/lounge", "dev/dc/dn/lounge", '9'),
                '13' : ("light/dn/dance-floor", "dev/dc/dn/lounge-central", '9'),
                '14' : ("light/dn/dance-floor", "dev/dc/dn/lounge", '6'),
                '15' : ("light/dn/entry", "dev/dc/dn/entry1", '5'),
                '16' : ("light/dn/ensuite", "dev/dc/dn/entry2", '4'),
                '17' : ("light/dn/toilet", "dev/dc/dn/entry2", '8'),
                '18' : ("light/up/bed1", "dev/dc/up/bed1", 'F'),
                '19' : ("light/up/bed2", "dev/dc/up/bed2", 'F'),
                '20' : ("light/up/bathroom", "dev/dc/up/bathroom", 'F'),
                '21' : ("light/up/hallway", "dev/dc/up/hallway", '6'),
                '22' : ("light/up/hallway", "dev/dc/up/lounge1", 'A'),
                '23' : ("light/up/toilet", "dev/dc/up/hallway", '1'),
                '24' : ("light/up/bed3", "dev/dc/up/bed3", 'F'),
                '25' : ("light/up/bed3", "dev/dc/up/stairs1", '2'),
                '26' : ("light/stairs", "dev/dc/up/stairs1", '5'),
                '27' : ("light/stairs", "dev/dc/up/stairs2", '3'),
                '28' : ("light/stairs", "dev/dc/dn/entry1", '2'),
                '29' : ("light/up/lounge", "dev/dc/up/lounge1", '5'),
                '30' : ("light/up/lounge", "dev/dc/up/lounge2", 'F'),
                '31' : ("light/up/lounge", "dev/dc/up/lounge3", '7'),
                '32' : ("light/up/lounge", "dev/dc/up/lounge4", 'F')
}

def main ():
    menuLoop()

def print_menu():       ## Your menu design here
    print(25 * "-" , "Kroby Configuration Tool" , 25 * "-")
    print("1. Upload node.hex to all devices")
    print("2. Set device 'friendly' names")
    print("3. Power cycle the CAN bus")
    print("4. Clear the LC's lighting zone subscriptions")
    print("5. Set the LC's lighting zone subscriptions")
    print("6. Print device to 'friendly' name table")
    print("7. Print the lighting zone table")
    print("8. Generate _nocanc.conf mqtt file")
    print("9. Named tuple testing")
    print("10. Exit")
    print((50+26) * "-")


def menuLoop():
    loop=True      

    while loop:             ## While loop which will keep going until loop = False
        print_menu()        ## Displays menu
        choice = input("Enter your choice: ")
        choice = int(choice)
        
        if choice==1:     
            uploadFW()
        elif choice==2:
            renameToFriendly()
        elif choice==3:
            powerCycleCAN()
        elif choice==4:
            clearLCSubs()
        elif choice==5:
            configLightZones()
        elif choice==6:
            print('\n', 10 * '*', ' Device to Friendly Name ', 10 * '*', '\n')
            for dev, name in nodeNames.items():
                print('{} -> {}'.format(dev, name))
            print('\n')
        elif choice==7:
            #template = "{0:40}|{1:40}|{2:5}"
            #for num, settings in lightingZones.items():
            #    print(template.format(settings))
            print('\n', 10 * '*', ' Lighting zone device mappings and channel mask ', 10 * '*', '\n')
            for num, settings in lightingZones.items():
                print(settings[0] + " -> " + settings[1] + " | " +  settings[2])
            print('\n')
        elif choice==8:
            genConfFile()
        elif choice==9:
            printNamedTuple()
        elif choice==10:
            print("Exit configuration tool...")
            loop=False
        else:
            input("Wrong option selection. Enter any key to try again..")

#
# Testing named tuples
#
def printNamedTuple():
    for x in nodeNames2:
        print(x.deviceID)
        print(x.friendlyName)


#
# Generate a new _nocanc.conf MQTT file
#
def genConfFile():
    # "w" will create new file, or over write existing file
    f = open("new_nocanc.conf", "w")

    newString = 'event-server = "{}"\n'.format(nocanEventServer)
    newString += 'auth-token = "{}"\n\n'.format(nocanAuthToken)

    newString += '[mqtt]\n'
    newString += 'mqtt-server = "{}"\n\n'.format(mqttServer)

    for num, settings in lightingZones.items():
        newString += '[[mqtt.publishers]]\n'
        newString += 'channel="{0}{1}"\n'.format(mqttPubPrefix, settings[0])
        newString += 'topic="{0}{1}{2}"\n\n'.format(mqttTopicPrefix, mqttPubPrefix, settings[0])

        newString += '[[mqtt.subscribers]]\n'
        newString += 'channel="{0}{1}"\n'.format(mqttSubPrefix, settings[0])
        newString += 'topic="{0}{1}{2}"\n'.format(mqttTopicPrefix, mqttSubPrefix, settings[0])
        newString += "\n\n"

    f.write(newString)
    f.close()
    print("\n\n\n\nnew_nocanc.conf file written\n\n\n\n") 


#
# Upload new FW to all nodes in network
#
def uploadFW():
    print("\nUpload node.hex...")
    for x in range(1,30):
        if (x != 22):
            print('\tUploading to Node: ' + str(x))
            subprocess.run(["nocanc.exe", "upload", '{}'.format(fileName), '{}'.format(x)])
    print("\tComplete")

#
# Set default node names to friendly / location names
#
def renameToFriendly():
    print("\nRename nodes to friendly names...")
    for node, name in nodeNames.items():
        print('\t' + node + ' renamed to ' +  name)
        subprocess.run(["nocanc.exe", "publish", '{}'.format(node), '{}'.format(krbRenameNode + name)])
        sleep(0.2)
    print("\tComplete")
    #delay a little to let Nodes write to flash
    sleep(1)

#
# Power cycle to get new node device names
#
def powerCycleCAN():
    print("\nPower cycle CAN bus to bring up new Node names...")
    subprocess.run(["nocanc.exe", "power", "off"])
    sleep(2)

    subprocess.run(["nocanc.exe", "power", "on"])
    print("\tWait " + str(bootDelay) + "s for CAN network to reboot")
    sleep(bootDelay)

#
# Clear exisitng light zone subscriptions
#
# use new node names to send krbClearLoadSub command to each node in network
def clearLCSubs():
    print("\nClear existing subscription in all Light Controllers...")
    for node, name in nodeNames.items():
        print('\tClearing ', name, ' lighting zone subscriptions')
        subprocess.run(["nocanc.exe", "publish", "{}".format('dev/' + name), "{}".format(krbClearLoadSubs)])
        sleep(0.2)
    print("\tComplete")

#
# Configure Lighting Zone
#
def configLightZones():
    print("\nConfigure lighting zone subscriptions...")
    for num, settings in lightingZones.items():
        print("Ligting zone: " + settings[0] + ", device:" + settings[1] + ", with led mask: " +  settings[2])
        subprocess.run(["nocanc.exe", "publish", "{}".format(settings[1]),
                        "{}".format(krbSetLoadSubNext + settings[2] + settings[0])])
        sleep(0.2)
    print("\tComplete")

if __name__ == "__main__":
    main()