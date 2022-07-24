dev: 
	go run cmd/main.go
deploy:
	env GOOS=linux GOARCH=arm GOARM=5 go build -o example ./example/main.go
	scp example pi@raspberrypi.local:~/
