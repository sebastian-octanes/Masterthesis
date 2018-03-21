begin
  server = listen(3003)
    while true
      sock = accept(server)
      @async while isopen(sock)
        println(readline(sock))
      end
  end
end

