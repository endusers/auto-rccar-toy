#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# The MIT License

# Copyright (c) 2023 Motoyuki Endo
# Released under the MIT license
# http://opensource.org/licenses/mit-license.php

# Copyright (c) 2001-2023 Python Software Foundation; All Rights Reserved

import sys
import threading
import rclpy
from PyQt5.QtWidgets import QApplication
from route_publisher.route_publisher import RoutePublisher


def main(args=None):
    app = QApplication( sys.argv )

    try:
        rclpy.init( args=args )

        node = RoutePublisher()
        node.createWidget()

        thread = threading.Thread( target=rclpy.spin, args=( node, ), daemon=True )
        thread.start()

        node.widget.show()

        while node.widget.isVisible():
            app.processEvents()

        #app.exec_()

    except KeyboardInterrupt:
        print( 'Keyboard interrupt exception Caught' )

    finally:
        node.nav2_client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        thread.join()

        sys.exit( 0 )


if __name__ == '__main__' :
    main()
