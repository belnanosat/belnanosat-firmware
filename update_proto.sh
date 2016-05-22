#!/bin/sh
protoc --plugin=protoc-gen-nanopb=nanopb/generator/protoc-gen-nanopb --nanopb_out=src/proto/ src/proto/telemetry.proto --proto_path=src/proto/
